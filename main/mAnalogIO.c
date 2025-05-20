#include "mAnalogIO.h"
#include "esp_log.h"

#include "driver/adc.h"
#define TAG "ADC"

static uint8_t _battery_channel = -1;
static uint8_t _ldr_channel = -1;

esp_err_t adc_init(int ldr_channel, int battery_channel)
{
    _battery_channel = battery_channel;
    _ldr_channel = ldr_channel;

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(battery_channel, ADC_ATTEN_DB_12);
    adc1_config_channel_atten(ldr_channel, ADC_ATTEN_DB_0);

    return ESP_OK;
}

static float AnalogRead(int channel)
{
    float sum = 0;
    for (size_t i = 0; i < 20; i++)
    {
        sum += adc1_get_raw(channel);
    }

    sum /= 20;

    ESP_LOGI(TAG, "analog read: %.3f", sum * 3.3 / 4069);

    return sum * 3.3 / 4069;
}

int ldr_is_dark(void)
{
    if (AnalogRead(_ldr_channel) > 0.1)
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

    if (AnalogRead(_battery_channel) > 1.75)
    {
        return 0;
    }

    return 1;
}