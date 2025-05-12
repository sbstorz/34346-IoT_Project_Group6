#ifndef MDIGITALIO_H
#define MDIGITALIO_H

#include "esp_err.h"
#include "driver/gpio.h"

esp_err_t led_init(gpio_num_t pin, int32_t level);
esp_err_t led_state_on(void);
esp_err_t led_state_off(void);

#endif // MDIGITALIO_H