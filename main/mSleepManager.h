#ifndef MSLEEPMANAGER_H
#define MSLEEPMANAGER_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"

// Application-level wake event type
typedef enum
{
    APP_WAKE_UNDEFINED_BOOT,  // Initial power-on or undefined wake reason
    APP_WAKE_ADXL_ACTIVITY,   // Woke due to ADXL345 activity detection (INT1)
    APP_WAKE_ADXL_INACTIVITY, // Woke due to ADXL345 inactivity detection (INT2)
    APP_WAKE_TIMER,           // Woke due to ESP32 timer
    APP_WAKE_USER_BUTTON,     // Woke due to Euser button press
    APP_WAKE_OTHER            // Woke due to other ESP32 sources or unknown EXT1 pin
} app_wake_event_t;

// ADXL Motion wake-up
typedef enum
{
    WAKE_NONE = 0x00,
    WAKE_ACTIVITY = 0x01,   // Wake on activity
    WAKE_INACTIVITY = 0x02, // Wake on Inactivity
} adxl_wake_source_t;

// Lora tranmission status type
typedef enum
{
    LORA_FAIL = -1,    // Transmission failed
    LORA_BUSY = 0,     // Transmission in progress
    LORA_TX_READY = 1, // Transmission succesfull, no downlink received
    LORA_RX_READY = 2, // Transmission sucesfull, downlink received, can be retrieved by calling TODO
} lora_status_t;

/**
 * @brief Initializes I2C communication and the ADXL345 sensor.
 *
 * This function should be called once at the beginning of app_main.
 * @param sda GPIO pin for SDA
 * @param scl GPIO pin for SCL
 * @return esp error code
 */
esp_err_t sm_init_adxl(gpio_num_t sda, gpio_num_t scl);

/**
 * @brief gets the system wakeup reason
 * @return application level wakup reason
 */
app_wake_event_t sm_get_wakeup_cause(void);

/**
 * @brief Enter deep sleep
 * @param motion_int_pin pin on which the motion interrupt is attached, can be `GPIO_NUM_NC` to disable wake-up source
 * @param user_button_pin pin on which the user button is attached, can be `GPIO_NUM_NC` to disable wake-up source
 * @param timer_duration_us timer based wake-up period, can be 0 to disable wake-up source
 * @return esp error code
 */
esp_err_t sm_deep_sleep(
    gpio_num_t motion_int_pin,
    gpio_num_t user_button_pin,
    uint64_t timer_duration_us);

esp_err_t sm_enable_adxl_wakeups(adxl_wake_source_t source);

esp_err_t sm_wait_tx_done(void);
esp_err_t sm_tx_state_if_due(uint8_t flags);
esp_err_t sm_tx_location(void);


lora_status_t sm_get_lora_status(void);

esp_err_t sm_get_rx_data(char *buf, size_t buf_size);

#endif // MSLEEPMANAGER_H