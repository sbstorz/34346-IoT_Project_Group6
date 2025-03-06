#ifndef MLED_H
#define MLED_H

#include "driver/gpio.h"
#include "esp_check.h"

void GPIO_configure_led(void);
void GPIO_toggle_led(void);
void GPIO_attach_ToggleHandler(void);



#endif