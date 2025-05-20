#ifndef MSLEEPMANAGER_H
#define MSLEEPMANAGER_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"

// Application-level wake event type
typedef enum
{
    APP_WAKE_UNDEFINED_BOOT,  // Initial power-on or undefined wake reason
    APP_WAKE_ADXL_ACTIVITY,   // Woke due to ADXL345 activity detection
    APP_WAKE_ADXL_INACTIVITY, // Woke due to ADXL345 inactivity detection
    APP_WAKE_TIMER,           // Woke due to RTC timer
    APP_WAKE_USER_BUTTON,     // Woke due to user button press
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

/**
 * @brief Enables accelerometer motion interrupt.
 *
 * The interrupt(s) passed as the argument are enabled, all other are disabled.
 * @param source One of WAKE_INACTIVITY, WAKE_ACTIVITY, WAKE_NONE. It is possible to enable
 * both WAKE_INACTIVITY and WAKE_ACTIVITY by bit wise OR'ing them.
 * @return esp error code
 */
esp_err_t sm_enable_adxl_wakeups(adxl_wake_source_t source);

/**
 * @brief Checks if a motion interrupt has been triggered on the accelerometer.
 *
 * The interrupt(s) passed as the argument are enabled, all other are disabled.
 * @param source One of WAKE_INACTIVITY, WAKE_ACTIVITY, WAKE_NONE. It is possible to enable
 * both WAKE_INACTIVITY and WAKE_ACTIVITY by bit wise OR'ing them.
 * @return esp error code
 */
int sm_get_adxl_int_status(adxl_wake_source_t source);

/**
 * @brief Blocking function waiting for the current LoRaWAN transmission to finish.
 *
 * The function waits for a signal from the LoRaWAN transmission thread either TX, RX or FAIL.
 * The function does not indicate and returns if a downlink message has been received.
 * @return ESP_OK when transmission ends with either RX or TX, or ESP_FAIL when the transmission failed.
 */
esp_err_t sm_wait_tx_done(void);

/**
 * @brief Checks if the LoRaWAN cooldown period has passed and a hence a new uplink message is due.
 * This function transmits the byte passed as an argument via LoRaWAN if the LoRaWAN cooldown period has passed.
 * The function is intended to uplink the single status byte during normal operation.
 * If a transmission is due it will be started in a seperate task. The state of this task can be retrieved
 * by calling `sm_get_lora_status()`. Hence this function is non-blocking.
 *
 * @param flags single byte to send uplink. The payload is not restricted to 1 byte but this is
 * sufficient for this application.
 * @return ESP_OK
 */
esp_err_t sm_tx_state_if_due(uint8_t flags);

/**
 * @brief Initiates the geolocation and sends the obtained location + status byte.
 * This function starts the GNSS geolocation process and will, after its completion, send the result
 * via LoRaWAN to the backend
 * The function is blocking until a GNSS solution has been found. There after the transmission is running
 * in a seperate task. The state of this task can be retrieved by calling `sm_get_lora_status()`.
 *
 * @param flags single byte to send uplink. The payload is not restricted to 1 byte but this is
 * sufficient for this application.
 * @return ESP_OK
 */
esp_err_t sm_tx_location(uint8_t flags);

/**
 * @brief Wrapper function that returns the status flag of a LoRaWAN Transmission thread.
 * This function retrieves the flags from the LoRaWAN Transmission thread set in the private
 * xEventGroup. This function can be used to poll the state and initiate actions based on it.
 * @return LORA_FAIL if the tranmission failed;
 * LORA_BUSY while the transmission is ongoing;
 * LORA_TX_READY if the transmission succesfully sent the Uplink and no Downlink has been received,
 * or no transmission has been initiated since the cooldown period has not passed;
 * LORA_RX_READY if the transmission succesfully sent the Uplink and
 * a Downlink has been received, the downlink message can be retrieved by sm_get_rx_data().
 */
lora_status_t sm_get_lora_status(void);

/**
 * @brief Returns the last LoRaWAN downlink message.
 * This function copies the LoRaWAN downlink message from the private RX buffer to the supplied
 * application buffer in a thread-safe manner.
 * @param buf pointer to buffer
 * @param buf_size size of provided buffer
 * @return ESP_ERR_INVALID_ARG if buf == NULL;
 * ESP_ERR_INVALID_SIZE if buf_size < RX_BUF_SIZE (10);
 * ESP_OK otherwise
 */
esp_err_t sm_get_rx_data(char *buf, size_t buf_size);

#endif // MSLEEPMANAGER_H