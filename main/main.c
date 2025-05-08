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

// Helper function to configure and enter the final deep sleep state (wake
// onactivity only)
static void enter_inactive_deep_sleep() {
    ESP_LOGI(TAG_MAIN, "Configuring deep sleep (wake on INT1 only)...");

    // Ensure ADXL345 interrupt pins are cleared immediately before sleep config
    uint8_t clear_int_source = adxl345_get_and_clear_int_source();
    ESP_LOGI(TAG_MAIN, "Cleared ADXL345 INT_SOURCE (0x%02x) during deep sleep.",
             clear_int_source);
    // Initializes the specified GPIO pin (ADXL345_INT1_GPIO) to be used as an
    // RTC IO pin.
    ESP_ERROR_CHECK(rtc_gpio_init(ADXL345_INT1_GPIO));

    // Sets the direction of the specified RTC GPIO pin (ADXL345_INT1_GPIO) to
    // input only. Since this pin will receive an interrupt signal from the
    // ADXL345 to wake the ESP32, the ESP32 needs to treat it as an input.
    ESP_ERROR_CHECK(
        rtc_gpio_set_direction(ADXL345_INT1_GPIO, RTC_GPIO_MODE_INPUT_ONLY));

    // To ensure the pin reads a stable LOW level when the interrupt is not
    // active, a pull-down resistor is enabled. This prevents the pin from
    // "floating" at an indeterminate voltage level
    ESP_ERROR_CHECK(rtc_gpio_pulldown_en(ADXL345_INT1_GPIO));

    // Since we enabled the pull-down resistor, we must disable the pull-up
    // resistor to avoid conflicting signals and ensure the pin is properly
    // pulled low when the external signal is not driving it high.
    ESP_ERROR_CHECK(rtc_gpio_pullup_dis(ADXL345_INT1_GPIO));

    // The hold feature can keep a pin's state fixed during sleep. For an input
    // pin being used as a wake-up source, we want it to accurately reflect the
    // level driven by the external sensor. Disabling the hold ensures the pin's
    // state is NOT artificially latched and can respond to changes on the
    // ADXL345's INT1 line.
    ESP_ERROR_CHECK(rtc_gpio_hold_dis(ADXL345_INT1_GPIO));

    // Deinitialize the INT2 GPIO pin (inactivity interrupt) as it's not used in
    // this mode
    rtc_gpio_deinit(ADXL345_INT2_GPIO);

    // Enable the external wake-up source for the INT1 GPIO pin
    const uint64_t ext_wakeup_pin_mask = (1ULL << ADXL345_INT1_GPIO);
    ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_mask,
                                                 ESP_EXT1_WAKEUP_ANY_HIGH));

    ESP_LOGI(TAG_MAIN, "Entering deep sleep (wake on activity only)...");
    esp_deep_sleep_start();
}

// Helper function to process wake-up cause and update RTC state variable (Refactored)
static void determine_wake_state(esp_sleep_wakeup_cause_t cause) {
    switch (cause) {
        case ESP_SLEEP_WAKEUP_EXT1: { // Woke from Deep Sleep via EXT1 (INT1 or INT2)
            uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
            if (wakeup_pin_mask & (1ULL << ADXL345_INT1_GPIO)) {
                ESP_LOGI(TAG_MAIN, "Woke up due to ACTIVITY (INT1)");
                rtc_is_currently_inactive = false; // Activity -> Active state
            } else if (wakeup_pin_mask & (1ULL << ADXL345_INT2_GPIO)) {
                // This path is unexpected if deep sleep only enables INT1 wake when inactive
                ESP_LOGW(TAG_MAIN, "Woke up due to INACTIVITY (INT2) from DEEP SLEEP - Unexpected!");
                rtc_is_currently_inactive = true; // Treat as Inactive state
            } else {
                ESP_LOGW(TAG_MAIN, "Woke up via EXT1, but unknown pin? Mask: %llx", wakeup_pin_mask);
                // Keep previous RTC state as fallback
            }
            break;
        }
        case ESP_SLEEP_WAKEUP_UNDEFINED: // Initial Power-On Boot
            ESP_LOGI(TAG_MAIN, "Initial power-on boot.");
            rtc_is_currently_inactive = true; // Default to Inactive state
            break;

        case ESP_SLEEP_WAKEUP_GPIO: // Woke from Light Sleep via GPIO (should be INT2)
            ESP_LOGI(TAG_MAIN, "Woke up from LIGHT sleep (INT2 - Inactivity)");
            rtc_is_currently_inactive = true; // Treat light sleep wake as transition to inactive
            break;
        default: // Catch-all for other/unexpected causes
            ESP_LOGI(TAG_MAIN, "Woke up via other cause (%d), keeping previous RTC state.", cause);
            // No state change, relies on the value stored in RTC memory
            break;
    }
}

// Helper function to initialize peripherals for the ACTIVE state
static void initialize_active_mode_peripherals() {
    ESP_LOGI(TAG_MAIN, "Initializing peripherals for ACTIVE state...");

    // I2C should already be initialized by app_main start
    adxl345_set_measure_mode();
    ESP_ERROR_CHECK(adxl345_config_activity_int(ACTIVITY_THRESHOLD_VALUE,
                                                ADXL345_INT1_PIN));
    ESP_ERROR_CHECK(adxl345_config_inactivity_int(
        INACTIVITY_THRESHOLD_VALUE, INACTIVITY_TIME_S, ADXL345_INT2_PIN));

    uint8_t temp_int_source;
    adxl345_read_reg(ADXL345_REG_INT_SOURCE,
                     &temp_int_source);  // Clear ADXL345 flags
    ESP_LOGI(TAG_MAIN,
             "Cleared ADXL345 INT_SOURCE (0x%02x) during active init.",
             temp_int_source);

    // Configure INT2 pin for light sleep GPIO wake-up
    gpio_config_t io_conf_int2_light_wake = {
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = (1ULL << ADXL345_INT2_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE};
    ESP_ERROR_CHECK(gpio_config(&io_conf_int2_light_wake));
    // Ensure INT1 is reset as it is not the primary wake source for light sleep
    gpio_reset_pin(ADXL345_INT1_GPIO);
}

// Helper function to configure and enter light sleep waiting for inactivity
// (INT2)
static void enter_light_sleep_waiting_for_inactivity() {
    ESP_LOGI(TAG_MAIN, "Preparing light sleep (wake on INT2 pin HIGH only)...");
    esp_sleep_disable_wakeup_source(
        ESP_SLEEP_WAKEUP_ALL);  // Disable previous deep sleep sources
    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
    ESP_ERROR_CHECK(
        gpio_wakeup_enable(ADXL345_INT2_GPIO, GPIO_INTR_HIGH_LEVEL));
    gpio_wakeup_disable(
        ADXL345_INT1_GPIO);  // Ensure INT1 doesn't wake from light sleep

    esp_light_sleep_start();
    // Execution resumes here after waking from light sleep (due to INT2)
}

void app_main(void) {
    boot_count++;
    ESP_LOGI(TAG_MAIN, "--- System Boot/Wake-up #%d ---", boot_count);

    vTaskDelay(pdMS_TO_TICKS(200));
    adxl345_i2c_master_init(SDA_IO_PIN, SCL_IO_PIN);

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

    determine_wake_state(cause);

    ESP_LOGI(TAG_MAIN, "Determined state for this cycle: %s",
             rtc_is_currently_inactive ? "INACTIVE" : "ACTIVE");

    // --- State Machine ---
    if (rtc_is_currently_inactive) {
        enter_inactive_deep_sleep();
    } else {
        // ACTIVE STATE
        initialize_active_mode_peripherals();
        ESP_LOGI(TAG_MAIN,
                 "Entering ACTIVE state loop (Work -> Light Sleep cycle)...");

        // main loop should include the real state machine
        while (!rtc_is_currently_inactive) {
            // Do periodic active work (optional)
            ESP_LOGI(TAG_MAIN, "Active loop: Doing work... Placeholder delay.");
            vTaskDelay(pdMS_TO_TICKS(2000));

            // Enter light sleep waiting for inactivity
            enter_light_sleep_waiting_for_inactivity();

            // --- Wake from Light Sleep ---
            // If code reaches here, it MUST have been woken by INT2
            ESP_LOGI(TAG_MAIN,
                     "Woke from light sleep - INACTIVITY (INT2) occurred!");
            rtc_is_currently_inactive = true;
            // The loop condition will now be false, loop will exit.
        }

        // --- Exit Active Loop ---
        ESP_LOGI(TAG_MAIN,
                 "Exited active loop because inactivity was detected.");
        enter_inactive_deep_sleep();
        ESP_LOGE(TAG_MAIN,
                 "Error: Reached end of app_main active state unexpectedly!");
        enter_inactive_deep_sleep();
    }
}