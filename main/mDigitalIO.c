#include "mDigitalIO.h"

static gpio_num_t _led_pin = GPIO_NUM_NC;

esp_err_t led_init(gpio_num_t pin,int32_t state)
{
    _led_pin = pin;

    // gpio_reset_pin(RST_PIN);
    /* Set the GPIO as a push/pull output */
    gpio_config_t cfg = {
        .pin_bit_mask = BIT64(_led_pin),
        .mode = GPIO_MODE_OUTPUT,
        // for powersave reasons, the GPIO should not be floating, select pullup
        .pull_up_en = true,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);

    if (state)
    {
        led_state_on();
    }else{
        led_state_off();
    }
    
    gpio_hold_dis(_led_pin);

    return ESP_OK;
}

esp_err_t led_state_on(void)
{
    gpio_set_level(_led_pin, 0);
    return ESP_OK;
}

esp_err_t led_state_off(void)
{
    gpio_set_level(_led_pin, 1);
    return ESP_OK;
}
