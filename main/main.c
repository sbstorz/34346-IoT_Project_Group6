#include <stdio.h>
#include <sys/time.h>
#include <time.h>

#include "mSleepManager.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mRN2483.h"
#include "mDigitalIO.h"
#include "mGNSS.h"
#include "mAnalogIO.h"

#define TAG "Main"
#define TX_BUF_SIZE 3 * 4 + 1

#define IS_INIT (1 << 0)
#define IS_STOLEN (1 << 1)
#define IN_MOTION (1 << 2)
#define LED_ON (1 << 3)
#define LOW_BAT (1 << 4)

#define LED_ON_DSLEEP_DURATION_S 5
#define LED_OFF_DSLEEP_DURATION_S 10
#define STOLEN_DSLEEP_DURATION_S 30
#define MOTION_DSLEEP_DURATION_S 6

typedef enum
{
    wakeup,
    idle,
    led_on,
    led_off,
    low_bat,
    stolen,
    light_auto,
    dsleep,
} state_t;

/* RTC Vars for main application logic */
static RTC_DATA_ATTR uint8_t state_flags = 0;

void app_main(void)
{

    state_t state = wakeup;
    bool wake_lock = 0;
    bool stop = 0;

    led_init(GPIO_NUM_13, state_flags & LED_ON);
    button_init(GPIO_NUM_12);
    adc_init(0, 3);

    if (sm_init_adxl(GPIO_NUM_19, GPIO_NUM_18) != ESP_OK)
    {
        ESP_LOGE(TAG, "FAILED to initialize the accelerometer");
        return;
    }

    if (gnss_init(UART_NUM_1, GPIO_NUM_22, GPIO_NUM_23, GPIO_NUM_21) != ESP_OK)
    {
        ESP_LOGE(TAG, "FAILED to initialize the GNSS Module");
        return;
    }

    ESP_LOGI(TAG, "Woken up, flags are: %#.2x", state_flags);
    if (!(state_flags & IS_INIT))
    {
        ESP_LOGI(TAG, "Initial Boot");
        state_flags |= IS_INIT;

        rn_init(UART_NUM_2, GPIO_NUM_5, GPIO_NUM_16, GPIO_NUM_27, 1024, true);

        ESP_ERROR_CHECK_WITHOUT_ABORT(rn_sleep());
        gnss_sleep();
    }
    else
    {
        rn_init(UART_NUM_2, GPIO_NUM_5, GPIO_NUM_16, GPIO_NUM_27, 1024, false);
    }

    while (!stop)
    {

        switch (state)
        {
        case wakeup:
        {
            if (state_flags & IS_STOLEN)
            {
                state = stolen;
                break;
            }

            app_wake_event_t app_event = sm_get_wakeup_cause();
            switch (app_event)
            {
            case APP_WAKE_UNDEFINED_BOOT:
                ESP_LOGI(TAG, "Wakeup processing: Undefined/Initial Boot.");
                state = idle;
                break;
            case APP_WAKE_ADXL_ACTIVITY:
                ESP_LOGI(TAG,
                         "Wakeup processing: ADXL Activity detected.");
                state_flags |= IN_MOTION;
                state = light_auto;
                break;
            case APP_WAKE_ADXL_INACTIVITY:
                ESP_LOGI(TAG,
                         "Wakeup processing: ADXL Inactivity detected.");
                state_flags &= ~IN_MOTION;
                state = led_off;
                break;
            case APP_WAKE_TIMER:
                ESP_LOGI(TAG, "Wakeup processing: Timer.");
                if (state_flags & IN_MOTION && !(state_flags & LED_ON))
                {
                    state = light_auto;
                }
                else
                {
                    state = idle;
                }
                break;
            case APP_WAKE_USER_BUTTON:
                ESP_LOGI(TAG, "Wakeup processing: User Button.");
                /* If LED is on, turn it off, else turn it on*/
                if (state_flags & LED_ON)
                {
                    state = led_off;
                }
                else
                {
                    state = led_on;
                }
                break;
            case APP_WAKE_OTHER:
            default:
                ESP_LOGI(TAG,
                         "Wakeup processing: Other ESP32 cause or Unknown EXT1.");
                state = idle; // Default fallback
                break;
            }

            if (battery_is_low())
            {
                ESP_LOGI(TAG, "Battery Low");
                state_flags |= LOW_BAT;
            }
            else
            {
                ESP_LOGI(TAG, "Battery not Low");
                state_flags &= ~LOW_BAT;
            }

            break; // Break from case wakeup
        }

        case light_auto:
            ESP_LOGI(TAG, "State: light_auto");
            if (ldr_is_dark())
            {
                ESP_LOGI(TAG, "dark");
                state = led_on;
            }
            else
            {
                ESP_LOGI(TAG, "light");
                state = idle;
            }

            break;

        case led_on:
            if (state_flags & LOW_BAT)
            {
                if (!led_is_blinking())
                {
                    ESP_LOGI(TAG, "Turn LED: BLINK");
                    led_start_blink();
                    sm_enable_adxl_wakeups(WAKE_INACTIVITY);
                    // DEBOUNCE delay
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                }
                wake_lock = 1;
            }
            else
            {
                ESP_LOGI(TAG, "Turn LED: ON");
                led_state_on();
                sm_enable_adxl_wakeups(WAKE_INACTIVITY);
                // DEBOUNCE delay
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }

            state_flags |= LED_ON;
            state = idle;

            break;

        case led_off:
            ESP_LOGI(TAG, "state: led_off");
            if (led_is_blinking())
            {
                ESP_LOGI(TAG, "LED STOP BLINK");
                led_stop_blink();
                wake_lock = 0;
            }
            else
            {
                ESP_LOGI(TAG, "LED OFF");
                led_state_off();
            }
            state_flags &= ~LED_ON;
            if (state_flags & IS_STOLEN)
            {
                state = stolen;
            }
            else
            {
                state = idle;
            }
            break;

        case idle:

            sm_tx_state_if_due(state_flags & LOW_BAT ? 0x01 : 0x00);

            if (state_flags & LOW_BAT && state_flags & LED_ON)
            {
                state = led_on;
                wake_lock = 1;
            }
            if (button_get_level())
            {
                button_wait_fEdge();
                if (state_flags & LED_ON)
                {
                    state = led_off;
                }
                else
                {
                    state = led_on;
                }
            }

            if (sm_get_adxl_int_status(WAKE_INACTIVITY))
            {
                if (state_flags & LED_ON)
                {
                    state_flags &= ~IN_MOTION;
                    state = led_off;
                }
            }

            switch (sm_get_lora_status())
            {
            case LORA_TX_READY:
                if (!(wake_lock))
                {
                    state = dsleep;
                }

                break;
            case LORA_RX_READY:

                char buf[10];
                sm_get_rx_data(buf, 10);
                ESP_LOG_BUFFER_HEXDUMP(TAG, buf, 10, ESP_LOG_INFO);
                if (buf[0] & 0x01)
                {
                    sm_enable_adxl_wakeups(WAKE_NONE);
                    led_state_off();
                    state_flags |= IS_STOLEN;
                    wake_lock = 0;
                    state = led_off;
                }
                else
                {
                    state_flags &= ~IS_STOLEN;
                    if (!(wake_lock))
                    {
                        state = dsleep;
                    }
                }

                break;
            case LORA_FAIL:
                if (!(wake_lock))
                {
                    state = dsleep;
                }
                break;
            default:
                break;
            }
            break;

        case stolen:

            sm_tx_location(state_flags & LOW_BAT ? 0x01 : 0x00);
            sm_wait_tx_done();
            switch (sm_get_lora_status())
            {
            case LORA_RX_READY:
                char buf[10];
                sm_get_rx_data(buf, 10);
                ESP_LOG_BUFFER_HEXDUMP(TAG, buf, 10, ESP_LOG_INFO);
                /* If stolen and receviced downlink indicating no longer stolen */
                if (!(buf[0] & 0x01))
                {
                    sm_enable_adxl_wakeups(WAKE_ACTIVITY);
                    state_flags &= ~IS_STOLEN;
                }
                break;
            case LORA_TX_READY:
            case LORA_FAIL:
            default:
                break;
            }
            state = dsleep;
            break;

        case dsleep:
            stop = 1;
            break;
        default:
            ESP_LOGW(TAG, "Unknown state: %d, going to sleep.", state);
            state = dsleep;
            break;
        }

        if (!stop)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    unsigned int dsleep_time_s = LED_OFF_DSLEEP_DURATION_S;
    if (state_flags & IS_STOLEN)
    {
        dsleep_time_s = STOLEN_DSLEEP_DURATION_S;
    }
    else if (state_flags & LED_ON)
    {
        dsleep_time_s = LED_ON_DSLEEP_DURATION_S;
    }
    else if (state_flags & IN_MOTION)
    {
        dsleep_time_s = MOTION_DSLEEP_DURATION_S;
    }

    if (button_get_level())
    {
        ESP_LOGI(TAG, "wait for f_edge");
        button_wait_fEdge();
        vTaskDelay(50 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "f_edge came");
    }

    gpio_hold_en(GPIO_NUM_13);
    gpio_hold_en(GPIO_NUM_27);
    if (state_flags & IS_STOLEN)
    {
        sm_deep_sleep(GPIO_NUM_NC, GPIO_NUM_NC, dsleep_time_s * 1000 * 1000);
    }
    else
    {
        sm_deep_sleep(GPIO_NUM_15, GPIO_NUM_12, dsleep_time_s * 1000 * 1000);
    }
}
