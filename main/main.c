#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include "esp_sleep.h"
#include "mRN2483.h"
#include "mGNSS.h"

#define TAG "Main"
#define TX_BUF_SIZE 3 * 4 + 1

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
    gnss_init(UART_NUM_1, GPIO_NUM_22, GPIO_NUM_23, GPIO_NUM_21);

    ESP_LOGI(TAG, "Woken up, flags are: %#.2x", state_flags);
    if (!(state_flags & (1 << 0)))
    {
        ESP_LOGI(TAG, "Initial Boot");
        state_flags |= (1 << 0);

        /* TX on GPIO 5 since this is pulled up after reset and during sleep, reset could be removed since MCU and Lora module are always restarted together*/
        rn_init(UART_NUM_2, GPIO_NUM_5, GPIO_NUM_16, GPIO_NUM_4, 1024, true);
        if (rn_init_otaa() != ESP_OK)
        {
            return;
        }
    }
    else
    {
        if (rn_init(UART_NUM_2, GPIO_NUM_5, GPIO_NUM_16, GPIO_NUM_4, 1024, false) != ESP_OK)
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
                state = idle;
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

            /* not stolen:  [FLG]               */
            /* stolen:      [LAT][LAT][LAT][LAT][LON][LON][LON][LON][HAC][HAC][HAC][HAC][FLG]*/

            char msg[TX_BUF_SIZE];

            /* if not stolen*/
            if (!(state_flags & (1 << 1)))
            {

                ESP_LOGI(TAG, "Sending TX: Battery only");
                msg[0] = 0 /*battery_get_state()*/;

                unsigned int port = 0;
                char rx_data[10];
                /* Send Uplink data */
                if (rn_wake() == ESP_OK)
                {
                    rn_tx(msg, 1, 1, true, rx_data, 10, &port);
                }
                if (port > 0)
                {
                    ESP_LOGI(TAG, "Received RX, port: %d, data:", port);
                    ESP_LOG_BUFFER_HEXDUMP(TAG, rx_data, strlen(rx_data), ESP_LOG_INFO);

                    /* Check flags in buffer
                     * if STOLEN flag is raised, save it in state flags
                     * if not STOLEN clear the HADFIX flag and STOLEN flag */
                    if (rx_data[0] & (1 << 0))
                    {
                        state_flags |= (1 << 1);
                    }
                    else
                    {
                        state_flags &= ~(1 << 1);
                        state_flags &= ~(1 << 2);
                    };
                }
            }
            /* If is STOLEN */
            if (state_flags & (1 << 1))
            {
                ESP_LOGI(TAG, "Acitvating GNSS");
                /* Init GNSS */
                int32_t latitudeX1e7, longitudeX1e7, hMSL, hAcc, vAcc;
                if (gnss_wake() == ESP_OK)
                {
                    /* If HADFIX, there has been a fix, execute faster position acquisition */
                    if (state_flags & (1 << 2))
                    {
                        ESP_LOGI(TAG,"Had fix, quick search");
                        /* If not succesfull with quick seach, clear HADFIX flag */
                        if(gnss_get_location(&latitudeX1e7, &longitudeX1e7, &hMSL, &hAcc, &vAcc, 60, false) != ESP_OK){
                            state_flags &= ~(1 << 2);
                        }
                    }
                    /* if not HADIFIX, either because newly stolen or failed quick acquisition, 
                    * prolonged search*/
                    if (!(state_flags & (1 << 2)))
                    {
                        ESP_LOGI(TAG,"Did not had fix, long search");
                        /* inital GNSS search (long time), max. 5 min.
                         * put to sleep while searching for GNSS. 
                         * If succesfull set HADFIX flag*/
                        if(gnss_get_location(&latitudeX1e7, &longitudeX1e7, &hMSL, &hAcc, &vAcc, 5*60, true) == ESP_OK){
                            state_flags |= (1 << 2);
                        }
                    }
                    /* Put GNSS back to sleep */
                    gnss_sleep();

                    ESP_LOGI(TAG, "I am here: https://maps.google.com/?q=%3.7f,%3.7f; Height: %.3f; Horizontal Uncertainty: %.3f; Vertical Uncertainty: %.3f ",
                             ((float)latitudeX1e7) / 10000000,
                             ((float)longitudeX1e7) / 10000000,
                             ((float)hMSL) / 1000,
                             ((float)hAcc) / 1000,
                             ((float)vAcc) / 1000);

                    /* populate msg buffer with FIX */
                    memcpy(msg, &latitudeX1e7, sizeof(int32_t));
                    memcpy(msg + 4, &longitudeX1e7, sizeof(int32_t));
                    memcpy(msg + 8, &hAcc, sizeof(int32_t));

                    ESP_LOGI(TAG, "Sending TX: Location + Battery");
                    msg[TX_BUF_SIZE-1] = 0 /*battery_get_state()*/;

                    ESP_LOG_BUFFER_HEXDUMP(TAG, msg, 13, ESP_LOG_INFO);

                    unsigned int port = 0;
                    char rx_data[10];
                    /* Send Uplink data */
                    if (rn_wake() == ESP_OK)
                    {
                        rn_tx(msg, TX_BUF_SIZE, 1, true, rx_data, 10, &port);
                    }

                    /* Parse buffer again*/
                    if (port > 0)
                    {
                        ESP_LOGI(TAG, "Received RX, port: %d, data:", port);
                        ESP_LOG_BUFFER_HEXDUMP(TAG, rx_data, strlen(rx_data), ESP_LOG_INFO);

                        /* Check flags in buffer*/
                        if (rx_data[0] & (1 << 0))
                        {
                            state_flags |= (1 << 1);
                        }
                        else
                        {
                            state_flags &= ~(1 << 1);
                            state_flags &= ~(1 << 2);
                        }
                    }
                }
            }

            state = dsleep;

            break;

        case dsleep:

            rn_sleep();

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
    gettimeofday(&sleep_enter_time, NULL);
    esp_deep_sleep_start();
}