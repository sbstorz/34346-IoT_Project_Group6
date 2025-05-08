#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "adxl345.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"  // For rtc_gpio_is_valid_gpio, rtc_gpio_deinit
#include "esp_log.h"
#include "esp_sleep.h"  // For sleep functions
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "uI2C.h"

// Fallback definition for linter if ESP_EXT1_WAKEUP_ANY_HIGH is not found
#ifndef ESP_EXT1_WAKEUP_ANY_HIGH
#define ESP_EXT1_WAKEUP_ANY_HIGH \
    1  // This is the typical enum value for this mode
#warning \
    "ESP_EXT1_WAKEUP_ANY_HIGH was not defined by esp_sleep.h. Using fallback definition (1). Check SDK/linter config."
#endif

static const char *TAG_MAIN = "APP_MAIN";

#define ADXL345_INT1_GPIO GPIO_NUM_12  // Activity
#define ADXL345_INT2_GPIO GPIO_NUM_14  // Inactivity
#define ACTIVITY_THRESHOLD_VALUE 80
#define INACTIVITY_THRESHOLD_VALUE 80
#define INACTIVITY_TIME_S 5

RTC_DATA_ATTR static bool rtc_is_currently_inactive = true;
RTC_DATA_ATTR static int boot_count = 0;

// Generic helper to configure RTC GPIO for deep sleep wake-up and enter deep sleep
static void configure_rtc_gpio_and_enter_deep_sleep(gpio_num_t wakeup_gpio, gpio_num_t other_gpio, const char *wake_event_name) {
    ESP_LOGI(TAG_MAIN, "Configuring deep sleep (wake on GPIO%d - %s)...", wakeup_gpio, wake_event_name);

    uint8_t clear_int_source = adxl345_get_and_clear_int_source();
    ESP_LOGI(TAG_MAIN, "Cleared ADXL345 INT_SOURCE (0x%02x) before deep sleep for %s.", clear_int_source, wake_event_name);

    ESP_ERROR_CHECK(rtc_gpio_init(wakeup_gpio));
    ESP_ERROR_CHECK(rtc_gpio_set_direction(wakeup_gpio, RTC_GPIO_MODE_INPUT_ONLY));
    ESP_ERROR_CHECK(rtc_gpio_pulldown_en(wakeup_gpio));
    ESP_ERROR_CHECK(rtc_gpio_pullup_dis(wakeup_gpio));
    ESP_ERROR_CHECK(rtc_gpio_hold_dis(wakeup_gpio));
    
    // Deinitialize the other interrupt pin to prevent unintended wake-ups or current draw
    if (rtc_gpio_is_valid_gpio(other_gpio)) { // Check if the other_gpio is a valid RTC GPIO
        rtc_gpio_deinit(other_gpio);
    }


    const uint64_t ext_wakeup_pin_mask = (1ULL << wakeup_gpio);
    ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_mask, ESP_EXT1_WAKEUP_ANY_HIGH));

    ESP_LOGI(TAG_MAIN, "Entering deep sleep (wake on %s only)...", wake_event_name);
    esp_deep_sleep_start();
}

// Helper function to configure and enter deep sleep, waking on ACTIVITY (INT1)
static void enter_deep_sleep_wake_on_activity() {
    configure_rtc_gpio_and_enter_deep_sleep(ADXL345_INT1_GPIO, ADXL345_INT2_GPIO, "activity");
}

// Helper function to configure and enter deep sleep, waking on INACTIVITY (INT2)
static void enter_deep_sleep_wake_on_inactivity() {
    configure_rtc_gpio_and_enter_deep_sleep(ADXL345_INT2_GPIO, ADXL345_INT1_GPIO, "inactivity");
}

// Helper function to process wake-up cause and update RTC state variable
static void determine_wake_state(esp_sleep_wakeup_cause_t cause) {
    switch (cause) {
        case ESP_SLEEP_WAKEUP_EXT1: {
            uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
            if (wakeup_pin_mask & (1ULL << ADXL345_INT1_GPIO)) { // ACTIVITY detected
                ESP_LOGI(TAG_MAIN, "Woke up due to ACTIVITY (INT1 on GPIO%d)", ADXL345_INT1_GPIO);
                rtc_is_currently_inactive = false; // Became active
            } else if (wakeup_pin_mask & (1ULL << ADXL345_INT2_GPIO)) { // INACTIVITY detected
                ESP_LOGI(TAG_MAIN, "Woke up due to INACTIVITY (INT2 on GPIO%d)", ADXL345_INT2_GPIO);
                rtc_is_currently_inactive = true; // Became inactive
            } else {
                ESP_LOGW(TAG_MAIN, "Woke up via EXT1, but unknown pin? Mask: 0x%llx. Keeping RTC state: %s",
                         wakeup_pin_mask, rtc_is_currently_inactive ? "INACTIVE" : "ACTIVE");
            }
            break;
        }
        case ESP_SLEEP_WAKEUP_UNDEFINED:
            ESP_LOGI(TAG_MAIN, "Initial power-on boot.");
            rtc_is_currently_inactive = true; // Default to Inactive state
            break;
        default:
            ESP_LOGI(TAG_MAIN, "Woke up via other cause (%d). Keeping RTC state: %s",
                     cause, rtc_is_currently_inactive ? "INACTIVE" : "ACTIVE");
            // For other causes, maintain the previous state stored in RTC memory.
            break;
    }
}

// Helper function to initialize ADXL345 interrupts for both activity and inactivity detection
static void setup_adxl345_interrupts() {
    ESP_LOGI(TAG_MAIN, "Setting up ADXL345 for Activity (INT1) and Inactivity (INT2) interrupts...");
    adxl345_set_measure_mode(); // Includes device ID check
    ESP_ERROR_CHECK(adxl345_config_activity_int(ACTIVITY_THRESHOLD_VALUE, ADXL345_INT1_PIN));
    ESP_ERROR_CHECK(adxl345_config_inactivity_int(INACTIVITY_THRESHOLD_VALUE, INACTIVITY_TIME_S, ADXL345_INT2_PIN));

    uint8_t temp_int_source = adxl345_get_and_clear_int_source();
    ESP_LOGI(TAG_MAIN, "Cleared ADXL345 INT_SOURCE (0x%02x) after ADXL345 setup.", temp_int_source);
}

void app_main(void) {
    boot_count++;
    ESP_LOGI(TAG_MAIN, "--- System Boot/Wake-up #%d ---", boot_count);

    // Delay for sensor stabilization after any reset
    vTaskDelay(pdMS_TO_TICKS(200)); 

    // Initialize I2C master once (ADXL345 will be configured later)
    adxl345_i2c_master_init(SDA_IO_PIN, SCL_IO_PIN);

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    determine_wake_state(cause);

    ESP_LOGI(TAG_MAIN, "Determined state for this cycle: %s",
             rtc_is_currently_inactive ? "INACTIVE" : "ACTIVE");

    // Configure ADXL345 interrupts (Activity on INT1, Inactivity on INT2)
    // This ensures the sensor is correctly signalling regardless of ESP32's previous sleep wake source
    setup_adxl345_interrupts();

    if (rtc_is_currently_inactive) {
        // INACTIVE STATE: Device is inactive, wait for an activity event to wake up.
        ESP_LOGI(TAG_MAIN, "State is INACTIVE. Entering deep sleep, waiting for activity (INT1)...");
        enter_deep_sleep_wake_on_activity();
    } else {
        // ACTIVE STATE: Device is active, wait for an inactivity event to go back to inactive.
        // The ADXL345's TIME_INACT register handles the inactivity duration.
        ESP_LOGI(TAG_MAIN, "State is ACTIVE. Entering deep sleep, waiting for inactivity (INT2)...");
        enter_deep_sleep_wake_on_inactivity();
    }
    // The lines below should not be reached as deep_sleep_start() resets the ESP32.
    ESP_LOGE(TAG_MAIN, "FATAL ERROR: Reached end of app_main! Should have reset in deep sleep.");
}