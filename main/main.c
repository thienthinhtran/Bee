#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dht11.h"

void app_main()
{
    // Initialize DHT11 sensor
    if (DHT11_init(GPIO_NUM_2) != ESP_OK) 
    {
        printf("Failed to initialize DHT11 sensor\n");
        return;
    }

    while (1)
    {
        float temperature, humidity;
        if (read_dht11(&temperature, &humidity))
        {
            printf("Temperature: %.2f°C, Humidity: %.2f%%\n", temperature, humidity);
        } 
        else
        {
            printf("Failed to read from DHT11\n");
        }

        // Delay for 2 seconds
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
