#ifndef MDIGITALIO_H
#define MDIGITALIO_H

#include "esp_err.h"
#include "driver/gpio.h"

esp_err_t led_init(gpio_num_t pin, const int32_t level);
esp_err_t led_state_on(void);
esp_err_t led_state_off(void);

esp_err_t led_stop_blink(void);
esp_err_t led_start_blink(void);
int led_is_blinking(void);


esp_err_t button_init(gpio_num_t pin);
int  button_get_state(void);
int button_get_level(void);
int button_had_rEdge(void);
esp_err_t button_wait_fEdge(void);


#endif // MDIGITALIO_H