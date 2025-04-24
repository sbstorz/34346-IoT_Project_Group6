#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include "esp_sleep.h"
#include "mRN2483.h"

#include "uI2C.h"

static const char *TAG = "main";

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
    lsleep
} state_t;

/* Vars that should be stored persistenly over deep sleep should be stored in RTC memory:
    - Last TXRX
    - Last Wakeuptime
    - flags (stolen,...)
    - initial boot flag
*/
static RTC_DATA_ATTR struct timeval sleep_enter_time;
static RTC_DATA_ATTR uint8_t state_flags = 0;

/* Configuration data that is rarely accessed should be stored in NVS*/
// dont know yet, but could be user (at runtime) configured WIFI credentials (FOR EXAMPLE)

/* Some Considerations:
Instead of continously running through the while loop, and repeatedly checking conditions in a state us the FreeRTOS tools like event bits to block the task.
This allows the FreeRTOS to allocate ticks to other tasks since the main task is not busy.
*/

void app_main(void)
{
    state_t state = wakeup;
    bool stop = 0;

    /* drivers which need to be initialized every boot:
        - rn_init
        - ...
    */

    /* check if this is the intital boot after power-down, if true:
        - rn_init_otaa

    */
    ESP_LOGI(TAG, "Woken up, flags are: %#.2x", state_flags);
    if (!state_flags & (1 << 0))
    {
        ESP_LOGI(TAG, "Initial Boot");
        state_flags |= (1 << 0);
        rn_init(UART_NUM_2, GPIO_NUM_17, GPIO_NUM_16, GPIO_NUM_4, 1024, true);

        if (rn_init_otaa() != ESP_OK)
        {
            return;
        }
    }
    else
    {
        if (rn_init(UART_NUM_2, GPIO_NUM_17, GPIO_NUM_16, GPIO_NUM_4, 1024, false) != ESP_OK)
        {
            return;
        }
        if (rn_wake() != ESP_OK)
        {
            return;
        }
    }



    while (!stop)
    {
        switch (state)
        {
        case wakeup:
            // get wake up reason
            // esp_sleep_get_wakeup_cause()
            // esp_sleep_get_ext1_wakeup_status()
            // if move,                                 --> goto: light_auto
            // if button,                               --> goto: light_on
            // if timer,                                --> goto: idle

            struct timeval now;
            gettimeofday(&now, NULL);
            int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;

            switch (esp_sleep_get_wakeup_cause())
            {
            case ESP_SLEEP_WAKEUP_TIMER:
            {
                ESP_LOGI(TAG, "Wake up from timer. Time spent in deep sleep: %dms\n", sleep_time_ms);
                break;
            }
            case ESP_SLEEP_WAKEUP_UNDEFINED:
            {
                ESP_LOGI(TAG, "Wake up not from deep sleep");
                break;
            }
            default:
                break;
            }

            state = txrx;
            break;

        case light_auto:
            // measure light outside
            // if dark,                                 --> goto: led_on
            // else,                                    --> goto: idle
            break;

        case idle:
            // check when last transmission was made
            // if led is on, turn off
            // if last transm. older than T hours,      --> goto: TXRX
            // else,                                    --> goto: dsleep
            break;

        case led_on:
            // turn led_on
            // configure wakeup sources: button,
            // and timeout (and inactivity?)            --> goto: lsleep
            break;

        case lsleep:
            // measure battery level
            // if low_batt,                             --> goto: led_blink
            // enter light sleep (or potentially deep sleep)
            // after wakeup execution continues here (in l. sleep only)
            // get wake reason
            // if wake reason button or inactivity,     --> goto: idle
            // else,                                    --> goto: lsleep
            break;

        case led_blink:
            // start led flash task
            // block and wait for button_press or inactivity
            // then,                                    --> goto: idle
            break;
        case txrx:
            // if !stolen, send battery
            // else, send battery + location
            // process RX
            // if RX contains new stolen,
            // send battery + location
            // reconfigure wakeups

            ESP_LOGI(TAG, "Sending TX");
            unsigned int port;
            char rx_data[10];
            if (rn_tx("Sunny Day!", 1, true, rx_data, 10, &port) == ESP_OK && port > 0)
            {
                ESP_LOGI(TAG, "Received RX, port: %d, data:", port);
                ESP_LOG_BUFFER_HEXDUMP(TAG, rx_data, strlen(rx_data), ESP_LOG_INFO);
            }
            ESP_ERROR_CHECK_WITHOUT_ABORT(rn_sleep());
            state = dsleep;

            break;

        case dsleep:
            const int wakeup_time_sec = 10;
            ESP_LOGI(TAG, "Enabling timer wakeup, %ds\n", wakeup_time_sec);
            ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000));
            stop = 1;
            break;
        default:
            break;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    // enter deep sleep
    esp_deep_sleep_start();
}
