#include "mSleepManager.h"

#include <sys/time.h> // For gettimeofday
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "esp_log.h"
// #include "uI2C.h"
#include "adxl345.h"

static const char *TAG = "SM"; // Tag for logging

// RTC data specific to this module's operation of determining sleep duration.
// Note: The main rtc_is_adxl_inactive flag is in main.c and passed by pointer.
static RTC_DATA_ATTR struct timeval sleep_enter_time;
static RTC_DATA_ATTR int sm_boot_count = 0;

// RTC memory could be avoided by using defines
static RTC_DATA_ATTR gpio_num_t _adxl_int1_pin = GPIO_NUM_NC;
static RTC_DATA_ATTR gpio_num_t _usr_button_pin = GPIO_NUM_NC;

// --- I2C and ADXL345 Initialization ---
esp_err_t sm_init_adxl(gpio_num_t sda, gpio_num_t scl)
{
    adxl_device_config_t adxl_config = {
        .activity_int_pin = ADXL345_INT1_PIN,
        .inactivity_int_pin = ADXL345_INT1_PIN,
        .activity_threshold = 40,
        .inactivity_threshold = 40,
        .inactivity_time_s = 30, /* TODO: Adjust*/
        .sda_pin = sda,
        .scl_pin = scl,
    };
    return adxl345_init(&adxl_config);
}

// --- ADXL345 Interrupt Setup ---
// void adxl_sm_setup_interrupts(void) {
//     ESP_LOGI(TAG,
//              "Setting up ADXL345 Activity/Inactivity interrupts...");
//     adxl345_set_measure_mode();  // From ADXL345 driver
//     // Assumes ADXL345_INT1_PIN and ADXL345_INT2_PIN are defined in adxl345.h
//     // for the driver
//     ESP_ERROR_CHECK(adxl345_config_activity_int(ADXL_ACTIVITY_THRESHOLD_VALUE,
//                                                 ADXL345_INT1_PIN));
//     ESP_ERROR_CHECK(adxl345_config_inactivity_int(
//         ADXL_INACTIVITY_THRESHOLD_VALUE, ADXL_INACTIVITY_TIME_S,
//         ADXL345_INT2_PIN));
//
//     uint8_t temp_int_source =
//         adxl345_get_and_clear_int_source();  // From ADXL345 driver
//     ESP_LOGI(TAG,
//              "Cleared ADXL345 INT_SOURCE (0x%02x) after ADXL345 setup.",
//              temp_int_source);
// }

// --- Deep Sleep Logic (Internal Helper) ---
// static void _configure_rtc_gpio_and_enter_deep_sleep(
//     gpio_num_t wakeup_gpio,
//     gpio_num_t other_gpio,
//     const char *wake_event_name,
//     bool enable_timer_too,
//     uint64_t timer_duration_us)
// {
//     ESP_LOGI(TAG, "Configuring deep sleep (wake on GPIO%d - %s)...",
//              wakeup_gpio, wake_event_name);

//     uint8_t clear_int_source = adxl345_get_and_clear_int_source();
//     ESP_LOGI(TAG,
//              "Cleared ADXL345 INT_SOURCE (0x%02x) before deep sleep for %s.",
//              clear_int_source, wake_event_name);

//     ESP_ERROR_CHECK(rtc_gpio_init(wakeup_gpio));
//     ESP_ERROR_CHECK(
//         rtc_gpio_set_direction(wakeup_gpio, RTC_GPIO_MODE_INPUT_ONLY));
//     ESP_ERROR_CHECK(
//         rtc_gpio_pulldown_en(wakeup_gpio)); // ADXL345 INT pins are active high
//     ESP_ERROR_CHECK(rtc_gpio_pullup_dis(wakeup_gpio));
//     ESP_ERROR_CHECK(rtc_gpio_hold_dis(wakeup_gpio));

//     if (rtc_gpio_is_valid_gpio(other_gpio))
//     {
//         rtc_gpio_deinit(other_gpio);
//     }

//     const uint64_t ext_wakeup_pin_mask = (1ULL << wakeup_gpio);
//     ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_mask,
//                                                  ESP_EXT1_WAKEUP_ANY_HIGH));

//     if (enable_timer_too && timer_duration_us > 0)
//     {
//         ESP_LOGI(TAG, "Also enabling timer wakeup for %llu us.",
//                  timer_duration_us);
//         ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(timer_duration_us));
//     }

//     ESP_LOGI(TAG, "Entering deep sleep (wake on %s %s)...",
//              wake_event_name,
//              (enable_timer_too && timer_duration_us > 0) ? "or timer" : "");
//     gettimeofday(&sleep_enter_time, NULL); // Record time before sleep
//     esp_deep_sleep_start();
// }

// --- Public Deep Sleep Functions ---
// void adxl_sm_enter_dsleep_wait_activity(bool *rtc_is_adxl_inactive_ptr,
//                                         bool also_enable_timer,
//                                         uint64_t timer_us)
// {
//     if (rtc_is_adxl_inactive_ptr)
//         *rtc_is_adxl_inactive_ptr = true; // System state is now considered
//                                           // inactive, waiting for activity
//     _configure_rtc_gpio_and_enter_deep_sleep(ADXL345_INT1_GPIO,
//                                              ADXL345_INT2_GPIO, "ADXL Activity",
//                                              also_enable_timer, timer_us);
// }

// void adxl_sm_enter_dsleep_wait_inactivity(bool *rtc_is_adxl_inactive_ptr,
//                                           bool also_enable_timer,
//                                           uint64_t timer_us)
// {
//     if (rtc_is_adxl_inactive_ptr)
//         *rtc_is_adxl_inactive_ptr = false; // System state is now considered
//                                            // active, waiting for inactivity
//     _configure_rtc_gpio_and_enter_deep_sleep(
//         ADXL345_INT2_GPIO, ADXL345_INT1_GPIO, "ADXL Inactivity",
//         also_enable_timer, timer_us);
// }

// --- Wake State Determination ---
// app_wake_event_t adxl_sm_determine_wake_state(bool *rtc_is_adxl_inactive_ptr)
// {
//     sm_boot_count++;
//     esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
//     struct timeval now;
//     gettimeofday(&now, NULL);
//     long sleep_time_ms = 0;
//     app_wake_event_t app_event = APP_WAKE_OTHER; // Default to other

//     if (cause != ESP_SLEEP_WAKEUP_UNDEFINED)
//     { // Only calculate sleep time if
//       // not initial boot
//         sleep_time_ms =
//             (now.tv_sec - sleep_enter_time.tv_sec) * 1000L +
//             (now.tv_usec - sleep_enter_time.tv_usec) / 1000L;
//     }
//
//     switch (cause)
//     {
//     case ESP_SLEEP_WAKEUP_EXT1:
//     {
//         uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
//         if (wakeup_pin_mask &
//             (1ULL << ADXL345_INT1_GPIO))
//         { // ACTIVITY detected
//             ESP_LOGI(TAG,
//                      "Woke: ADXL ACTIVITY (INT1). Sleep time: %ld ms",
//                      sleep_time_ms);
//             if (rtc_is_adxl_inactive_ptr)
//                 *rtc_is_adxl_inactive_ptr = false;
//             app_event = APP_WAKE_ADXL_ACTIVITY;
//         }
//         else if (wakeup_pin_mask &
//                  (1ULL << ADXL345_INT2_GPIO))
//         { // INACTIVITY detected
//             ESP_LOGI(TAG,
//                      "Woke: ADXL INACTIVITY (INT2). Sleep time: %ld ms",
//                      sleep_time_ms);
//             if (rtc_is_adxl_inactive_ptr)
//                 *rtc_is_adxl_inactive_ptr = true;
//             app_event = APP_WAKE_ADXL_INACTIVITY;
//         }
//         else
//         {
//             ESP_LOGW(TAG,
//                      "Woke: EXT1, unknown pin (0x%d). Sleep time: %ld ms. "
//                      "ADXL RTC state unchanged.",
//                      __builtin_ffsll(wakeup_pin_mask) - 1, sleep_time_ms);
//             app_event =
//                 APP_WAKE_OTHER; // Or a more specific APP_WAKE_EXT1_UNKNOWN
//         }
//         break;
//     }
//     case ESP_SLEEP_WAKEUP_TIMER:
//         ESP_LOGI(
//             TAG,
//             "Woke: Timer. Sleep time: %ld ms. ADXL RTC state unchanged.",
//             sleep_time_ms);
//         app_event = APP_WAKE_TIMER;
//         break;
//     case ESP_SLEEP_WAKEUP_UNDEFINED:
//         ESP_LOGI(TAG,
//                  "Woke: Initial boot/Undefined. ADXL RTC state set to "
//                  "INACTIVE.");
//         if (rtc_is_adxl_inactive_ptr)
//             *rtc_is_adxl_inactive_ptr = true;
//         app_event = APP_WAKE_UNDEFINED_BOOT;
//         break;
//     default: // Other specific ESP32 wake causes (Touch, UART, etc.)
//         ESP_LOGI(TAG,
//                  "Woke: Other ESP32 cause (%d). Sleep time: %ld ms. ADXL "
//                  "RTC state unchanged.",
//                  cause, sleep_time_ms);
//         app_event = APP_WAKE_OTHER;
//         break;
//     }

//     if (rtc_is_adxl_inactive_ptr)
//     {
//         ESP_LOGI(TAG, "Current ADXL RTC state: %s",
//                  *rtc_is_adxl_inactive_ptr ? "INACTIVE" : "ACTIVE");
//     }
//     return app_event;
// }

// ################### NEW IMPLEMENTATION ###########################

app_wake_event_t sm_get_wakeup_cause(void)
{
    sm_boot_count++;
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    struct timeval now;
    gettimeofday(&now, NULL);
    long sleep_time_ms = 0;
    app_wake_event_t app_event = APP_WAKE_OTHER; // Default to other

    if (cause != ESP_SLEEP_WAKEUP_UNDEFINED)
    { // Only calculate sleep time if
      // not initial boot
        sleep_time_ms =
            (now.tv_sec - sleep_enter_time.tv_sec) * 1000L +
            (now.tv_usec - sleep_enter_time.tv_usec) / 1000L;
    }

    switch (cause)
    {
    case ESP_SLEEP_WAKEUP_EXT1:
    {
        uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
        if (wakeup_pin_mask & (1ULL << _adxl_int1_pin))
        {
            // no error handling
            uint8_t int_src = adxl345_get_and_clear_int_source();
            if (int_src & ADXL345_INT_SOURCE_INACTIVITY)
            {
                ESP_LOGI(TAG,
                         "Woke: ADXL INT (INT1). Source: Inactivity. Sleep time: %ld ms",
                         sleep_time_ms);

                adxl345_enable_activity_int();
                adxl345_disable_inactivity_int();
                adxl345_get_and_clear_int_source();
                app_event = APP_WAKE_ADXL_INACTIVITY;
            }
            else if (int_src & ADXL345_INT_SOURCE_ACTIVITY)
            {
                ESP_LOGI(TAG,
                         "Woke: ADXL INT (INT1). Source: Activity. Sleep time: %ld ms",
                         sleep_time_ms);
                adxl345_enable_inactivity_int();
                adxl345_disable_activity_int();
                adxl345_get_and_clear_int_source();
                app_event = APP_WAKE_ADXL_ACTIVITY;
            }
        }
        else if (wakeup_pin_mask & (1ULL << _usr_button_pin))
        {
            ESP_LOGW(TAG,
                     "Woke: User Button. Sleep time: %ld ms",
                     sleep_time_ms);
                     
            adxl345_enable_inactivity_int();
            adxl345_disable_activity_int();
            adxl345_get_and_clear_int_source();
            app_event = APP_WAKE_USER_BUTTON;
        }
        else
        {
            ESP_LOGW(TAG,
                     "Woke: EXT1, unknown pin (0x%llx). Sleep time: %ld ms. "
                     "ADXL RTC state unchanged.",
                     wakeup_pin_mask, sleep_time_ms);
            app_event = APP_WAKE_OTHER; // Or a more specific APP_WAKE_EXT1_UNKNOWN
        }
        break;
    }
    case ESP_SLEEP_WAKEUP_TIMER:
        ESP_LOGI(
            TAG,
            "Woke: Timer. Sleep time: %ld ms.",
            sleep_time_ms);
        app_event = APP_WAKE_TIMER;
        break;
    case ESP_SLEEP_WAKEUP_UNDEFINED:
        ESP_LOGI(TAG,
                 "Woke: Initial boot/Undefined. ADXL configure wake-up on: ACTIVITY");
        adxl345_enable_activity_int();
        adxl345_disable_inactivity_int();
        adxl345_get_and_clear_int_source();
        app_event = APP_WAKE_UNDEFINED_BOOT;
        break;
    default: // Other specific ESP32 wake causes (Touch, UART, etc.)
        ESP_LOGI(TAG,
                 "Woke: Other ESP32 cause (%d). Sleep time: %ld ms. ADXL "
                 "RTC state unchanged.",
                 cause, sleep_time_ms);
        app_event = APP_WAKE_OTHER;
        break;
    }

    return app_event;
}

esp_err_t sm_deep_sleep(
    gpio_num_t motion_int_pin,
    gpio_num_t user_button_pin,
    uint64_t timer_duration_us)
{

    _adxl_int1_pin = motion_int_pin;
    _usr_button_pin = user_button_pin;

    // uint8_t clear_int_source = adxl345_get_and_clear_int_source();
    // ESP_LOGI(TAG,
    //          "Cleared ADXL345 INT_SOURCE (0x%02x) before deep sleep for %s.",
    //          clear_int_source, wake_event_name);

    // ESP_ERROR_CHECK(rtc_gpio_init(wakeup_gpio));
    // ESP_ERROR_CHECK(
    // rtc_gpio_set_direction(wakeup_gpio, RTC_GPIO_MODE_INPUT_ONLY));
    // ESP_ERROR_CHECK(
    // rtc_gpio_pulldown_en(wakeup_gpio)); // ADXL345 INT pins are active high
    rtc_gpio_pullup_dis(motion_int_pin);
    rtc_gpio_pulldown_en(motion_int_pin);
    rtc_gpio_pullup_dis(user_button_pin);
    rtc_gpio_pulldown_en(user_button_pin);
    // ESP_ERROR_CHECK(rtc_gpio_hold_dis(wakeup_gpio));

    const uint64_t ext_wakeup_pin_mask = (1ULL << motion_int_pin) | (1ULL << user_button_pin);
    ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_mask,
                                                 ESP_EXT1_WAKEUP_ANY_HIGH));

    if (timer_duration_us > 0)
    {
        esp_sleep_enable_timer_wakeup(timer_duration_us);
    }

    ESP_LOGI(TAG, "Device Sleepy: Going to sleep waking on EXT1 on Accel. Interrupt on GPIO %d, and "
                  "User button on GPIO %d. And timer in %lld s",
             motion_int_pin, user_button_pin, timer_duration_us / 1000000);

    gettimeofday(&sleep_enter_time, NULL); // Record time before sleep
    esp_deep_sleep_start();
}