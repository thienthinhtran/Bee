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

#define BUTTON_1_GPIO GPIO_NUM_18
#define BUTTON_2_GPIO GPIO_NUM_19

// Biến lưu trữ tần suất gửi dữ liệu (đơn vị: giây)
int dataSendFrequency = 1;
int printfFrequency = 1; // Thời gian giữa các lần printf (đơn vị: giây)
int uartSendReceiveFrequency = 1; // Thời gian giữa các lần gửi/nhận UART (đơn vị: giây)

// Biến lưu trữ cấp độ tần suất
int freqLevel = 1;

// Biến xác định trạng thái hoạt động
int isRunning = 1;

QueueHandle_t button_event_queue; // Hàng đợi để xử lý sự kiện nút bấm

void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(button_event_queue, &gpio_num, NULL);
}

void process_keyboard_command(char cmd) {
    switch (cmd) {
        case '1':
            // Thay đổi trạng thái hoạt động (bật/tắt)
            isRunning = !isRunning;
            if (isRunning) {
                printf("Resumed operation.\n");
            } else {
                printf("Paused operation.\n");
            }
            break;
        case '2':
            // Tăng cấp độ tần suất
            freqLevel++;
            if (freqLevel > 4) {
                freqLevel = 1; // Quay lại cấp độ thấp nhất nếu đang ở cấp độ cao nhất
            }
            // Thay đổi tần suất gửi dữ liệu
            printf("Pre-level: %d, Current-level: %d\n", freqLevel, dataSendFrequency);

            // Cập nhật tần suất dựa trên cấp độ
            switch (freqLevel) {
                case 1:
                    dataSendFrequency = 1;
                    printfFrequency = 1;
                    uartSendReceiveFrequency = 1;
                    break;
                case 2:
                    dataSendFrequency = 5;
                    printfFrequency = 5;
                    uartSendReceiveFrequency = 5;
                    break;
                case 3:
                    dataSendFrequency = 10;
                    printfFrequency = 10;
                    uartSendReceiveFrequency = 10;
                    break;
                case 4:
                    dataSendFrequency = 15;
                    printfFrequency = 15;
                    uartSendReceiveFrequency = 15;
                    break;
            }
            break;
        default:
            // Lệnh không được hỗ trợ
            printf("Unsupported command: %c\n", cmd);
            break;
    }
}

void send_uart_data(const char *data) {
    const int len = strlen(data);
    uart_write_bytes(UART_NUM, data, len);
}

void send_button_press_uart(const char *button) {
    char message[50];
    snprintf(message, sizeof(message), "Button %s pressed\n", button);
    send_uart_data(message);
}

char read_keypad() {
    int button_1_state = -1;
    int button_2_state = -1;

    // Đặt chân BUTTON_1_GPIO thành giá trị nền cao (pull-up)
    //gpio_set_pull_mode(BUTTON_1_GPIO, GPIO_PULLUP_ONLY);

    // Đặt chân BUTTON_2_GPIO thành giá trị nền thấp (pull-down)
    //gpio_set_pull_mode(BUTTON_2_GPIO, GPIO_PULLDOWN_ONLY);

    // Đọc trạng thái nút
    button_1_state = gpio_get_level(BUTTON_1_GPIO);
    button_2_state = gpio_get_level(BUTTON_2_GPIO);
    printf("KeyPad read: %d %d\n", butt`on_1_state, button_2_state);
    
    if (button_1_state == 0) {
        printf("Button 1\n");
        send_button_press_uart("1");
        return 1;
    } else if (button_2_state == 0) {
        printf("Button 2\n");
        send_button_press_uart("2");
        return 2;
    }

    return 0;
}

void button_task(void* arg) {
    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(button_event_queue, &io_num, portMAX_DELAY)) {
            // Nút bấm được nhấn, xử lý sự kiện
            char key = read_keypad();
            // if (key != '\0') {
            //     process_keyboard_command(key);
            //     vTaskDelay(500 / portTICK_PERIOD_MS); // Debounce delay
            // }
        }
        vTaskDelay(500 / portTICK_PERIOD_MS); // Debounce delay
    }
}

void app_main(void) {
    // Set the DHT pin
    setDHTgpio(4); // Replace with your desired pin

    // Set up LED pin
    esp_rom_gpio_pad_select_gpio(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    // Configure GPIOs for buttons
    gpio_config_t button_config = {
        .pin_bit_mask = (1ULL << BUTTON_1_GPIO) | (1ULL << BUTTON_2_GPIO),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_LOW_LEVEL, // Trigger on rising or falling edge
        .pull_up_en = GPIO_PULLUP_ENABLE, // Tắt pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE// Tắt pull-down
    };
    gpio_config(&button_config);

    // Create a queue to handle button events
    button_event_queue = xQueueCreate(10, sizeof(uint32_t));

    // Install ISR service for buttons
    gpio_install_isr_service(0);
    // Hook ISR handler to the buttons
    gpio_isr_handler_add(BUTTON_1_GPIO, gpio_isr_handler, (void*) BUTTON_1_GPIO);
    gpio_isr_handler_add(BUTTON_2_GPIO, gpio_isr_handler, (void*) BUTTON_2_GPIO);

    // // Start button task
    // xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);

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

    // Start button task
    xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);
    
    while (1) {
        if (isRunning) {
            int ret = readDHT();
            errorHandler(ret);

            if (ret == DHT_OK) {
                float temperature = getTemperature();
                float humidity = getHumidity();

                printf("Temperature: %.2f°C\n", temperature);
                vTaskDelay(printfFrequency * 1000 / portTICK_PERIOD_MS);

                printf("Humidity: %.2f%%\n", humidity);
                vTaskDelay(printfFrequency * 1000 / portTICK_PERIOD_MS);

                // Control LED based on temperature (example)
                if (temperature < 20.0) {
                    gpio_set_level(LED_GPIO, 1); // Turn on LED
                } else {
                    gpio_set_level(LED_GPIO, 0); // Turn off LED
                }

                // Send "Already received" message via UART
                send_uart_data("Already received\n");
                vTaskDelay(uartSendReceiveFrequency * 1000 / portTICK_PERIOD_MS);
            } else {
                // Print an error message if reading from DHT11 fails
                printf("DHT11 Reading Error\n");
            }

            vTaskDelay(dataSendFrequency * 1000 / portTICK_PERIOD_MS); // Delay based on dataSendFrequency
        } else {
            vTaskDelay(500 / portTICK_PERIOD_MS); // Debounce delay
        }
    }
}
