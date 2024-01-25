#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "DHT11.h"

void app_main(void) {
    // Set the DHT pin
    setDHTgpio(4); // Replace with your desired pin

    while (1) {
        int ret = readDHT();
        errorHandler(ret);

        if (ret == DHT_OK) {
            printf("Temperature: %.2f°C\n", getTemperature());
            printf("Humidity: %.2f%%\n", getHumidity());
        }

        vTaskDelay(2000 / portTICK_PERIOD_MS); // Adjust the delay as needed
    }
}
