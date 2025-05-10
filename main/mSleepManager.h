#ifndef MSLEEPMANAGER_H
#define MSLEEPMANAGER_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"

// ADXL345 Pin and Configuration Definitions
// #define ADXL345_INT1_GPIO GPIO_NUM_12  // Activity Interrupt
// #define ADXL345_INT2_GPIO GPIO_NUM_14  // Inactivity Interrupt
// #define ADXL_ACTIVITY_THRESHOLD_VALUE 80
// #define ADXL_INACTIVITY_THRESHOLD_VALUE 80
// #define ADXL_INACTIVITY_TIME_S 5

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
 * @brief Configures ADXL345 interrupts for activity and inactivity detection.
 *
 * Assumes ADXL345 has been initialized via adxl_sm_init_i2c_and_adxl.
 * Calls adxl345_set_measure_mode(), adxl345_config_activity_int(),
 * adxl345_config_inactivity_int(), and adxl345_get_and_clear_int_source()
 * from the ADXL345 driver.
 */
void adxl_sm_setup_interrupts(void);

/**
 * @brief Determines the system wake-up cause and updates ADXL-related RTC state.
 *
 * @param rtc_is_adxl_inactive_ptr Pointer to the RTC variable storing whether the
 *                                 system considers itself in an ADXL-inactive state.
 *                                 This function will update the value pointed to.
 * @return app_wake_event_t The application-level interpretation of the wake event.
 */
app_wake_event_t adxl_sm_determine_wake_state(bool *rtc_is_adxl_inactive_ptr);

/**
 * @brief Enters deep sleep, configured to wake on ADXL345 activity (INT1).
 *
 * @param rtc_is_adxl_inactive_ptr Pointer to the RTC variable. Will be set to false
 *                                 if this function is called, as it implies we are
 *                                 now waiting for activity.
 * @param also_enable_timer If true, a timer wakeup source will also be enabled.
 * @param timer_us Duration for the timer wakeup in microseconds.
 */
void adxl_sm_enter_dsleep_wait_activity(bool *rtc_is_adxl_inactive_ptr, bool also_enable_timer, uint64_t timer_us);

/**
 * @brief Enters deep sleep, configured to wake on ADXL345 inactivity (INT2).
 *
 * @param rtc_is_adxl_inactive_ptr Pointer to the RTC variable. Will be set to true
 *                                 if this function is called, as it implies we are
 *                                 now waiting for inactivity to transition to an inactive state.
 * @param also_enable_timer If true, a timer wakeup source will also be enabled.
 * @param timer_us Duration for the timer wakeup in microseconds.
 */
void adxl_sm_enter_dsleep_wait_inactivity(bool *rtc_is_adxl_inactive_ptr, bool also_enable_timer, uint64_t timer_us);


app_wake_event_t sm_get_wakeup_cause(void);

esp_err_t sm_deep_sleep(
    gpio_num_t motion_int_pin,
    gpio_num_t user_button_pin,
    uint64_t timer_duration_us);

#endif // MSLEEPMANAGER_H