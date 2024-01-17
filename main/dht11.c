// dht11.c
#include "dht11.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "freertos/portmacro.h"
#include "freertos/semphr.h"

static int wait_for_gpio(int level, TickType_t timeout) {
    while (gpio_get_level(DHT11_PIN) != level) {
        if (timeout == 0) {
            return 0; // Timeout
        }
        vTaskDelay(pdMS_TO_TICKS(1));
        if (timeout > 0) {
            timeout--;
        }
    }
    return 1; // Thành công
}

esp_err_t DHT11_init(gpio_num_t pin) {
        // Check if the GPIO pin is valid
        if (pin < GPIO_NUM_0 || pin >= GPIO_NUM_MAX) {
            return ESP_ERR_INVALID_ARG;
        }

        // Set up the GPIO pin for DHT11
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << pin),
            .mode = GPIO_MODE_OUTPUT,
            .intr_type = GPIO_INTR_DISABLE,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
        };
        gpio_config(&io_conf);

        // Set the initial state
        gpio_set_level(pin, 1);

        return ESP_OK;
    }

int read_dht11(float *temperature, float *humidity) 
{
    gpio_set_direction(DHT11_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(DHT11_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(18));
    gpio_set_level(DHT11_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(40));

    gpio_set_direction(DHT11_PIN, GPIO_MODE_INPUT);

    if (!wait_for_gpio(0, 80) || !wait_for_gpio(1, 80)) 
    {
        return 0; // Lỗi, không có phản hồi
    }

    uint8_t data[5] = {0};

    for (int i = 0; i < 5; ++i) 
    {
        uint8_t byte = 0;
        for (int j = 7; j >= 0; --j) 
        {
            if (!wait_for_gpio(1, 40)) 
            {
                return 0; // Lỗi, không có phản hồi
            }

            int level = gpio_get_level(DHT11_PIN);
            if (level == 0) 
            {
                // Bit 0, không cần xử lý gì cả
            } 
            
            else 
            {
                // Bit 1, set bit tương ứng trong byte
                byte |= (1 << j);
                while (gpio_get_level(DHT11_PIN) == 1) 
                {
                    // Đợi cảm biến chuyển về mức thấp
                }
            }
        }
        data[i] = byte;
    }
    // Kiểm tra checksum
    if ((uint8_t)(data[0] + data[1] + data[2] + data[3]) == data[4]) 
    {
        // Tính toán nhiệt độ và độ ẩm từ dữ liệu đọc được
        *humidity = (((uint16_t)data[0] << 8) | data[1]) / 10.0;
        *temperature = (((uint16_t)data[2] << 8) | data[3]) / 10.0;


        return 1; // Đọc thành công
    } 
    else
    {
        return 0; // Lỗi, checksum không khớp
    }
}
