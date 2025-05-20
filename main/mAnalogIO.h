#include "esp_err.h"

/**
 * @brief Initializes the ADC used for measuring brightness and battery voltage.
 * This function initializes the ESP32s ADC1 with two provided channels for measuring the voltage drop over the LDR
 * to infer brightness and the battery voltage to infer the battery SoC. The ADC channel and corresponding GPIO
 * are as follows: channel_0 = PIN_36, channel_1 = PIN_37, channel_2 = PIN_38, channel_3 = PIN_39
    channel_4 = PIN_32, channel_5 = PIN_33, channel_6 = PIN_34, channel_7 = PIN_35
 * @param ldr_channel the ADC1 channel the ldr is connected to.
 * @param battery_channel the ADC1 channel the battery is connected to.
 * @return ESP_OK
 */
esp_err_t adc_init(int ldr_channel, int battery_channel);

/**
 * @brief checks if the battery is low.
 * This function compares the battery voltage after the voltage divider with an internal threshold.
 * if the voltage is less than 1.75 V the battery is deemed to be low in charge.
 * @return 1 if the battery is low, 0 otherwise.
 */
int battery_is_low(void);

/**
 * @brief checks if it is dark outside.
 * This function compares the ldr voltage with an internal threshold.
 * if the voltage is grater than 0.1 V it is deemed dark outside.
 * @return 1 if it is dark, 0 otherwise.
 */
int ldr_is_dark(void);
