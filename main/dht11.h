#ifndef DHT11_H
#define DHT11_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define DHT11_PIN GPIO_NUM_2

esp_err_t DHT11_init(gpio_num_t pin);
int read_dht11(float *temperature, float *humidity);

#endif /* DHT11_H */
