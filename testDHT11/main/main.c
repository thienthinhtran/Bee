#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "DHT11.h"
#include "driver/uart.h"
#include "freertos/queue.h"
#include <string.h>

#define LED_GPIO 12
#define UART_NUM UART_NUM_2  // Chọn UART2
#define TXD_PIN (GPIO_NUM_17) // Chọn chân TX2
#define RXD_PIN (GPIO_NUM_16) // Chọn chân RX2
#define BUF_SIZE (1024)

void send_uart_data(const char *data) {
    const int len = strlen(data);
    uart_write_bytes(UART_NUM, data, len);
}

void app_main(void) {
    // Set the DHT pin
    setDHTgpio(4); // Replace with your desired pin

    // Set up LED pin
    esp_rom_gpio_pad_select_gpio(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    // Initialize and configure UART
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Install the UART driver (no FIFO)
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);

    while (1) {
        int ret = readDHT();
        errorHandler(ret);

        if (ret == DHT_OK) {
            float temperature = getTemperature();
            float humidity = getHumidity();

            printf("Temperature: %.2f°C\n", temperature);
            printf("Humidity: %.2f%%\n", humidity);

            // Control LED based on temperature (example)
            if (temperature < 20.0) {
                gpio_set_level(LED_GPIO, 1); // Turn on LED
            } else {
                gpio_set_level(LED_GPIO, 0); // Turn off LED
            }

            // Send "Already received" message via UART
            send_uart_data("Already received\n");
        } else {
            // Print an error message if reading from DHT11 fails
            printf("DHT11 Reading Error\n");
        }

        vTaskDelay(2000 / portTICK_PERIOD_MS); // Adjust the delay as needed
    }
}
