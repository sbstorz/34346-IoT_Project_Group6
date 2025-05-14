#include "esp_err.h"

esp_err_t adc_init(int ldr_pin, int battery_pin);
int battery_is_low(void);
int ldr_is_dark(void);
