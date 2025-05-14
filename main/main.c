#include <stdio.h>
#include <sys/time.h>
#include <time.h>

#include "mSleepManager.h"
#include "driver/gpio.h"
// #include "driver/i2c.h"
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
#define LED_ON (1 << 3)
#define LOW_BAT (1 << 4)

#define LED_ON_DSLEEP_DURATION_S 5
#define LED_OFF_DSLEEP_DURATION_S 10
#define STOLEN_DSLEEP_DURATION_S 30

typedef enum
{
    wakeup,
    idle,
    led_on,
    led_off,
    led_blink,
    low_bat,
    txrx,
    stolen,
    light_auto,
    dsleep,
    lsleep
} state_t;

/* RTC Vars for main application logic */
static RTC_DATA_ATTR uint8_t state_flags = 0;

/* Configuration data that is rarely accessed should be stored in NVS*/
// dont know yet, but could be user (at runtime) configured WIFI credentials
// (FOR EXAMPLE)

void app_main(void)
{

    state_t state = wakeup;
    bool wake_lock = 0;
    bool stop = 0;
    unsigned int dsleep_time_s = state_flags & LED_ON ? LED_ON_DSLEEP_DURATION_S : LED_OFF_DSLEEP_DURATION_S;

    led_init(GPIO_NUM_13, state_flags & LED_ON);
    button_init(GPIO_NUM_4);
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

    /* TX on GPIO 5 since this is pulled up after reset and during sleep, reset could be removed since MCU and Lora module are always restarted together*/
    rn_init(UART_NUM_2, GPIO_NUM_5, GPIO_NUM_16, GPIO_NUM_27, 1024, true);

    ESP_LOGI(TAG, "Woken up, flags are: %#.2x", state_flags);
    if (!(state_flags & IS_INIT))
    {
        ESP_LOGI(TAG, "Initial Boot");
        state_flags |= IS_INIT;

        rn_sleep();
        gnss_sleep();
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
                state = light_auto;
                break;
            case APP_WAKE_ADXL_INACTIVITY:
                ESP_LOGI(TAG,
                         "Wakeup processing: ADXL Inactivity detected.");
                state = led_off;
                break;
            case APP_WAKE_TIMER:
                ESP_LOGI(TAG, "Wakeup processing: Timer.");
                state = idle;
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
                state_flags |= LOW_BAT;
            }
            else
            {
                state_flags &= ~LOW_BAT;
            }

            sm_tx_state_if_due(state_flags & LOW_BAT ? 0x01 : 0x00);
            break;                // Break from case wakeup
        }

        case light_auto:
            ESP_LOGI(TAG, "State: light_auto");
            if (ldr_is_dark())
            {
                state = led_on;
            }
            else
            {
                state = idle;
            }

            break;

        case led_on:
            ESP_LOGI(TAG, "Turn LED: ON");
            if (state_flags & LOW_BAT)
            {
                if (!led_is_blinking())
                {
                    led_start_blink();
                    wake_lock = 1;
                }
            }
            else
            {
                led_state_on();
                dsleep_time_s = LED_ON_DSLEEP_DURATION_S;
            }

            state_flags |= LED_ON;
            state = idle;

            break;

        case led_off:
            ESP_LOGI(TAG, "Turn LED: OFF");
            if (led_is_blinking())
            {
                led_stop_blink();
                wake_lock = 0;
            }
            else
            {
                led_state_off();
            }
            state_flags &= ~LED_ON;
            dsleep_time_s = LED_OFF_DSLEEP_DURATION_S;
            if (state_flags & IS_STOLEN)
            {
                state = stolen;
            }
            else
            {
                state = idle;
            }
            break;

        case led_blink:

            ESP_LOGI(TAG, "State: led_blink");
            if (!led_is_blinking())
            {
                led_start_blink();
                state_flags |= LED_ON;
                wake_lock = 1;
            }
            state = idle;

            break;

        case idle:

            // if (button_had_rEdge())
            if (button_get_level())
            {
                button_wait_fEdge();
                vTaskDelay(100 / portTICK_PERIOD_MS);
                if (state_flags & LED_ON)
                {
                    state = led_off;
                }
                else
                {
                    state = led_on;
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
            // ESP_LOGI(TAG, "idle, next state: %d", state);
            break;

        case stolen:

            sm_tx_location();
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
            dsleep_time_s = STOLEN_DSLEEP_DURATION_S;
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
        sm_deep_sleep(GPIO_NUM_15, GPIO_NUM_4, dsleep_time_s * 1000 * 1000);
    }
}
