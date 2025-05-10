#include <stdio.h>
#include <sys/time.h>
#include <time.h>

#include "mSleepManager.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mRN2483.h"
#include "uUART.h"

#define TAG "Main"

typedef enum
{
    wakeup,
    idle,
    led_on,
    led_blink,
    low_bat,
    txrx,
    stolen,
    light_auto,
    dsleep,
} state_t;

/* RTC Vars for main application logic */
static RTC_DATA_ATTR uint8_t state_flags = 0;


/* Configuration data that is rarely accessed should be stored in NVS*/
// dont know yet, but could be user (at runtime) configured WIFI credentials
// (FOR EXAMPLE)

/* Some Considerations:
Instead of continously running through the while loop, and repeatedly checking
conditions in a state us the FreeRTOS tools like event bits to block the task.
This allows the FreeRTOS to allocate ticks to other tasks since the main task is
not busy.
*/

void app_main(void)
{
    state_t state = wakeup;
    bool stop = 0;

    /* Drivers */
    // Initialize ADXL345 and its I2C interface
    // delay for stabilization
    vTaskDelay(pdMS_TO_TICKS(500));

    if(sm_init_adxl(GPIO_NUM_21, GPIO_NUM_22) != ESP_OK){
        ESP_LOGE(TAG,"FAILED to initialize the accelerometer");
        return;
    }
    /* End of Drivers */

    // esp_sleep_wakeup_cause_t initial_cause_for_lora =
    // esp_sleep_get_wakeup_cause(); if (initial_cause_for_lora ==
    // ESP_SLEEP_WAKEUP_UNDEFINED && !(state_flags & (1 << 0))) {
    //     ESP_LOGI(TAG, "Initial Boot for LoRa OTAA");
    //     rn_init(UART_NUM_2, GPIO_NUM_5, GPIO_NUM_16, GPIO_NUM_4, 1024, true);
    //     if (rn_init_otaa() != ESP_OK) { return; }
    //     state_flags |= (1 << 0);
    // } else {
    //     ESP_LOGI(TAG, "Waking from sleep for LoRa (ABP or already joined)");
    //     rn_init(UART_NUM_2, GPIO_NUM_5, GPIO_NUM_16, GPIO_NUM_4, 1024,
    //     false); if (rn_wake() != ESP_OK) { /* Handle error */ }
    // }

    while (!stop)
    {
        ESP_LOGI(TAG, "Current State: %d, ADXL RTC Inactive: %s", state,
                 rtc_is_adxl_inactive ? "Yes" : "No");
        switch (state)
        {
        case wakeup:
        {
            app_wake_event_t app_event =
                sm_get_wakeup_cause();
            switch (app_event)
            {
            case APP_WAKE_UNDEFINED_BOOT:
                ESP_LOGI(TAG,
                         "Wakeup processing: Undefined/Initial Boot.");
                state =
                    light_auto; // Example: Initial action after boot
                break;
            case APP_WAKE_ADXL_ACTIVITY:
                ESP_LOGI(TAG,
                         "Wakeup processing: ADXL Activity detected.");
                // rtc_is_adxl_inactive is now false
                state = light_auto;
                break;
            case APP_WAKE_ADXL_INACTIVITY:
                ESP_LOGI(
                    TAG,
                    "Wakeup processing: ADXL Inactivity detected.");
                // rtc_is_adxl_inactive is now true
                state = idle;
                break;
            case APP_WAKE_TIMER:
                ESP_LOGI(TAG, "Wakeup processing: Timer.");
                // rtc_is_adxl_inactive state is preserved by
                // adxl_sm_determine_wake_state for timer wakes
                state = txrx; // Example: Periodic LoRa transmit
                break;
            case APP_WAKE_OTHER:
            default:
                ESP_LOGI(TAG,
                         "Wakeup processing: Other ESP32 cause or "
                         "Unknown EXT1.");
                state = idle; // Default fallback
                break;
            }
            break; // Break from case wakeup
        }

        case light_auto: // Example: Motion detected, turn on light
                         // (simulated)
            ESP_LOGI(TAG, "State: light_auto (ADXL was active)");
            vTaskDelay(pdMS_TO_TICKS(1000)); // Simulate light automation
            ESP_LOGI(TAG,
                     "light_auto finished, preparing for dsleep (wait for "
                     "ADXL inactivity).");
            // rtc_is_adxl_inactive is already false from wake state
            // determination
            state = dsleep;
            break;

        case idle: // Example: System is idle, perhaps after inactivity or
                   // timed task
            ESP_LOGI(TAG, "State: idle");
            // Decide if it's time for a periodic TX or just wait for motion
            // For now, assume it goes to deep sleep waiting for activity
            // rtc_is_adxl_inactive is already true from wake state or set
            // before dsleep
            state = dsleep;
            break;

        case led_on: // Example: Manual LED on, then wait for inactivity
            ESP_LOGI(TAG, "State: led_on");
            vTaskDelay(pdMS_TO_TICKS(3000)); // Simulate LED on
            ESP_LOGI(TAG,
                     "LED on period finished, will dsleep waiting for ADXL "
                     "inactivity.");
            rtc_is_adxl_inactive =
                false; // Explicitly set state before dsleep if needed
            state = dsleep;
            break;

            // case lsleep: // Potentially obsolete
            //    break;

        case led_blink: // Example: Low battery warning
            ESP_LOGI(TAG, "State: led_blink");
            vTaskDelay(pdMS_TO_TICKS(3000)); // Simulate blinking
            ESP_LOGI(TAG, "LED blink finished, going to idle/dsleep.");
            state = idle;
            break;

        case txrx: // Perform LoRaWAN transmission
            ESP_LOGI(TAG, "State: txrx");
            unsigned int port;
            char rx_data[10]; // Adjust size as needed
            if (rn_tx("TX Data!", 1, true, rx_data, sizeof(rx_data) - 1,
                      &port) == ESP_OK &&
                port > 0)
            {
                ESP_LOGI(TAG, "Received RX, port: %d, data:", port);
                ESP_LOG_BUFFER_HEXDUMP(TAG, rx_data, strlen(rx_data),
                                       ESP_LOG_INFO);
            }
            ESP_ERROR_CHECK_WITHOUT_ABORT(rn_sleep());
            ESP_LOGI(TAG,
                     "TX finished, will dsleep waiting for ADXL activity.");
            // rtc_is_adxl_inactive should be true to wait for activity
            // after TX
            state = dsleep;
            break;

        case dsleep:
        {
            const int periodic_wakeup_time_sec =
                60 * 5; // Example: 5 minutes for periodic TX
            ESP_LOGI(TAG, "State: dsleep. ADXL RTC Inactive Flag: %s",
                     rtc_is_adxl_inactive ? "Yes" : "No");
            if (rtc_is_adxl_inactive)
            {
                ESP_LOGI(
                    TAG,
                    "dsleep: Waiting for ADXL Activity OR Timer (%d s)",
                    periodic_wakeup_time_sec);
                // adxl_sm_enter_dsleep_wait_activity(
                //     &rtc_is_adxl_inactive, true,
                //     periodic_wakeup_time_sec * 1000000ULL);
            }
            else
            {
                ESP_LOGI(TAG,
                         "dsleep: Waiting for ADXL Inactivity (no periodic "
                         "timer)");
                // If active, we only care about when it becomes inactive.
                // ADXL_INACTIVITY_TIME_S handles timeout.
                // adxl_sm_enter_dsleep_wait_inactivity(&rtc_is_adxl_inactive,
                //                                      false, 0);
            }
            stop =
                1; // Will call esp_deep_sleep_start() in ADXL SM functions
            break;
        }
        default:
            ESP_LOGW(TAG, "Unknown state: %d, going to idle.", state);
            state = idle;
            break;
        }
        if (!stop)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    esp_deep_sleep_start();
}