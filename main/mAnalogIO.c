#include "mAnalogIO.h"

#include "driver/adc.h"
// #include "esp_adc/adc_oneshot.h"
// static adc_oneshot_unit_handle_t adc1_handle;

/*
channel_0 = PIN_36, channel_1 = PIN_37, channel_2 = PIN_38, channel_3 = PIN_39
channel_4 = PIN_32, channel_5 = PIN_33, channel_6 = PIN_34, channel_7 = PIN_35
#define Battery_Channel 6
#define LDR_Channel 4
*/

static uint8_t _battery_pin = -1;
static uint8_t _ldr_pin = -1;

esp_err_t adc_init(int ldr_pin, int battery_pin)
{
    _battery_pin = battery_pin;
    _ldr_pin = ldr_pin;

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(battery_pin, ADC_ATTEN_DB_12);
    adc1_config_channel_atten(ldr_pin, ADC_ATTEN_DB_0);

    return ESP_OK;
}

static float AnalogRead(int channel)
{
    uint8_t ANALOG_CHANNEL = channel;

    float sum = 0;
    for (size_t i = 0; i < 20; i++)
    {
        sum += adc1_get_raw(ANALOG_CHANNEL);
    }

    sum /= 20;

    return sum * 3.3 / 4069;
}

int ldr_is_dark(void)
{
    if (AnalogRead(_ldr_pin) > 1.5)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

int battery_is_low(void)
{
    if (AnalogRead(_battery_pin) > 1.75)
    // if (AnalogRead(_battery_pin) > 10.75)
    {
        return 0;
    }
    else
    {
        return 1;
    }

}