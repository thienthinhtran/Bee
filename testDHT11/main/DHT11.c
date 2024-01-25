#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "DHT11.h"

static const char* TAG = "DHT";

int DHTgpio = 4; // Default DHT pin = 4
float humidity = 0.;
float temperature = 0.;

void setDHTgpio(int gpio)
{
    DHTgpio = gpio;
}

float getHumidity() { return humidity; }
float getTemperature() { return temperature; }

void errorHandler(int response)
{
    switch (response)
    {
    case DHT_TIMEOUT_ERROR:
        ESP_LOGE(TAG, "Sensor Timeout\n");
        break;

    case DHT_CHECKSUM_ERROR:
        ESP_LOGE(TAG, "Checksum error\n");
        break;

    case DHT_OK:
        break;

    default:
        ESP_LOGE(TAG, "Unknown error\n");
    }
}

int getSignalLevel(int usTimeOut, bool state)
{
    int uSec = 0;
    while (gpio_get_level(DHTgpio) == state)
    {
        if (uSec > usTimeOut)
            return -1;

        ++uSec;
        esp_rom_delay_us(1); // uSec delay
    }

    return uSec;
}

#define MAXdhtData 5

int readDHT()
{
    int uSec = 0;

    uint8_t dhtData[MAXdhtData];
    uint8_t byteInx = 0;
    uint8_t bitInx = 7;

    for (int k = 0; k < MAXdhtData; k++)
        dhtData[k] = 0;

    gpio_set_direction(DHTgpio, GPIO_MODE_OUTPUT);
    gpio_set_level(DHTgpio, 0);
    esp_rom_delay_us(18000);

    gpio_set_direction(DHTgpio, GPIO_MODE_INPUT);
    uSec = getSignalLevel(160, 1);
    if (uSec < 0)
        return DHT_TIMEOUT_ERROR;

    uSec = getSignalLevel(160, 0);
    if (uSec < 0)
        return DHT_TIMEOUT_ERROR;

    uSec = getSignalLevel(160, 1);
    if (uSec < 0)
        return DHT_TIMEOUT_ERROR;

    for (int k = 0; k < 40; k++)
    {
        uSec = getSignalLevel(50, 0);
        if (uSec < 0)
            return DHT_TIMEOUT_ERROR;

        uSec = getSignalLevel(75, 1);
        if (uSec < 0)
            return DHT_TIMEOUT_ERROR;

        if (uSec > 28)
        {
            dhtData[byteInx] |= (1 << bitInx);
        }

        if (bitInx == 0)
        {
            bitInx = 7;
            ++byteInx;
        }
        else
            bitInx--;
    }

    humidity = dhtData[0] * 1.0;
    temperature = dhtData[2] * 1.0;

    if (dhtData[2] & 0x80)
        temperature *= -1;

    if (dhtData[4] == ((dhtData[0] + dhtData[1] + dhtData[2] + dhtData[3]) & 0xFF))
        return DHT_OK;
    else
        return DHT_CHECKSUM_ERROR;
}
