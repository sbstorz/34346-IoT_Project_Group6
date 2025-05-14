#include "mSleepManager.h"

#include <sys/time.h> // For gettimeofday
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "esp_log.h"
#include "adxl345.h"
#include "mRN2483.h"
#include "mGNSS.h"

#define LORA_FAIL_BIT BIT0
#define LORA_TX_READY_BIT BIT1
#define LORA_RX_READY_BIT BIT2
#define TX_BUF_SIZE 3 * 4 + 1
#define RX_BUF_SIZE 10
#define HAD_FIX (1 << 2)
#define LORA_COOLDOWN_S 20

static const char *TAG = "SM"; // Tag for logging

static EventGroupHandle_t s_lora_event_group = NULL;
static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

/* this is private to this file (static) */
typedef struct
{
    char buffer[TX_BUF_SIZE]; /*!< pointer to heap where message is stored*/
    size_t tx_length;         /*!< length of msg*/
} lora_msg_t;

static char _rx_data[RX_BUF_SIZE];

// RTC data specific to this module's operation of determining sleep duration.
static RTC_DATA_ATTR struct timeval sleep_enter_time;
static RTC_DATA_ATTR struct timeval last_lora_tx_time =  {.tv_sec = 0, .tv_usec = 0};

// RTC memory could be avoided by using defines
static RTC_DATA_ATTR gpio_num_t _adxl_int1_pin = GPIO_NUM_NC;
static RTC_DATA_ATTR gpio_num_t _usr_button_pin = GPIO_NUM_NC;
static RTC_DATA_ATTR uint8_t state_flags = 0;

static void rn_tx_task(void *params)
{
    lora_msg_t *msg = (lora_msg_t *)params;

    unsigned int port = 0;
    char rx_buf[RX_BUF_SIZE];
    /* Send Uplink data */
    if (rn_wake() == ESP_OK)
    {
        if (rn_tx(msg->buffer, msg->tx_length, 1, true, rx_buf, RX_BUF_SIZE, &port) == ESP_OK)
        {
            gettimeofday(&last_lora_tx_time, NULL);
        }else{
            xEventGroupSetBits(s_lora_event_group, LORA_FAIL_BIT);
        }
    }
    else
    {
        xEventGroupSetBits(s_lora_event_group, LORA_FAIL_BIT);
    }
    if (port > 0)
    {
        ESP_LOGI(TAG, "Received RX, port: %d, data:", port);
        ESP_LOG_BUFFER_HEXDUMP(TAG, rx_buf, RX_BUF_SIZE, ESP_LOG_INFO);

        taskENTER_CRITICAL(&spinlock);
        memcpy(_rx_data, rx_buf, RX_BUF_SIZE);
        taskEXIT_CRITICAL(&spinlock);

        xEventGroupSetBits(s_lora_event_group, LORA_RX_READY_BIT);
    }
    else
    {
        xEventGroupSetBits(s_lora_event_group, LORA_TX_READY_BIT);
    }

    free(msg);

    vTaskDelete(NULL);
}

static esp_err_t tx(lora_msg_t *msg)
{
    xTaskCreate(      // Use xTaskCreate() in vanilla FreeRTOS
        rn_tx_task,   // Function to be called
        "rn_tx_task", // Name of task
        1024 * 5,     // Stack size (bytes in ESP32, words in FreeRTOS)
        msg,          // Parameter to pass to function
        5,            // Task priority (0 to configMAX_PRIORITIES - 1)
        NULL);        // Task handle

    return ESP_OK;
}

static esp_err_t get_location_msg(lora_msg_t *msg)
{
    ESP_LOGI(TAG, "Acitvating GNSS");
    /* Init GNSS */
    int32_t latitudeX1e7, longitudeX1e7, hMSL, hAcc, vAcc;
    if (gnss_wake() != ESP_OK)
    {
        memset(msg->buffer, 0, msg->tx_length);
        return ESP_FAIL;
    }

    /* If HADFIX, there has been a fix, execute faster position acquisition */
    if (state_flags & HAD_FIX)
    {
        ESP_LOGI(TAG, "Had fix, quick search");
        /* If not succesfull with quick seach, clear HADFIX flag */
        if (gnss_get_location(&latitudeX1e7, &longitudeX1e7, &hMSL, &hAcc, &vAcc, 60, false) != ESP_OK)
        {
            state_flags &= ~HAD_FIX;
        }
    }
    /* if not HADIFIX, either because newly stolen or failed quick acquisition,
     * prolonged search*/
    if (!(state_flags & HAD_FIX))
    {
        ESP_LOGI(TAG, "Did not had fix, long search");
        /* inital GNSS search (long time), max. 5 min.
         * put to sleep while searching for GNSS.
         * If succesfull set HADFIX flag*/
        if (gnss_get_location(&latitudeX1e7, &longitudeX1e7, &hMSL, &hAcc, &vAcc, 5 * 60, true) == ESP_OK)
        {
            state_flags |= HAD_FIX;
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
    memcpy(msg->buffer, &latitudeX1e7, sizeof(int32_t));
    memcpy(msg->buffer + 4, &longitudeX1e7, sizeof(int32_t));
    memcpy(msg->buffer + 8, &hAcc, sizeof(int32_t));

    if (state_flags & HAD_FIX)
    {
        return ESP_OK;
    }
    return ESP_FAIL;

}

esp_err_t sm_init_adxl(gpio_num_t sda, gpio_num_t scl)
{
    adxl_device_config_t adxl_config = {
        .activity_int_pin = ADXL345_INT1_PIN,
        .inactivity_int_pin = ADXL345_INT1_PIN,
        .activity_threshold = 40,
        .inactivity_threshold = 40,
        .inactivity_time_s = 30,
        .sda_pin = sda,
        .scl_pin = scl,
    };
    return adxl345_init(&adxl_config);
}

app_wake_event_t sm_get_wakeup_cause(void)
{
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

                sm_enable_adxl_wakeups(WAKE_ACTIVITY);
                app_event = APP_WAKE_ADXL_INACTIVITY;
            }
            else if (int_src & ADXL345_INT_SOURCE_ACTIVITY)
            {
                ESP_LOGI(TAG,
                         "Woke: ADXL INT (INT1). Source: Activity. Sleep time: %ld ms",
                         sleep_time_ms);

                sm_enable_adxl_wakeups(WAKE_INACTIVITY);
                app_event = APP_WAKE_ADXL_ACTIVITY;
            }
        }
        else if (wakeup_pin_mask & (1ULL << _usr_button_pin))
        {
            ESP_LOGW(TAG,
                     "Woke: User Button. Sleep time: %ld ms",
                     sleep_time_ms);

            sm_enable_adxl_wakeups(WAKE_INACTIVITY);
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

        sm_enable_adxl_wakeups(WAKE_ACTIVITY);
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

    uint64_t ext_wakeup_pin_mask = 0x00;

    if (motion_int_pin != GPIO_NUM_NC)
    {
        rtc_gpio_pullup_dis(motion_int_pin);
        rtc_gpio_pulldown_en(motion_int_pin);
        ext_wakeup_pin_mask |= (1ULL << motion_int_pin);
    }
    if (user_button_pin != GPIO_NUM_NC)
    {
        rtc_gpio_pullup_dis(user_button_pin);
        rtc_gpio_pulldown_en(user_button_pin);

        ext_wakeup_pin_mask |= (1ULL << user_button_pin);
    }

    if (ext_wakeup_pin_mask)
    {
        esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_mask,
                                     ESP_EXT1_WAKEUP_ANY_HIGH);
    }

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

esp_err_t sm_enable_adxl_wakeups(adxl_wake_source_t source)
{
    esp_err_t rc = ESP_FAIL;
    if (source & BIT(0))
    {
        rc = adxl345_enable_activity_int();
    }
    else
    {
        rc = adxl345_disable_activity_int();
    }

    if (rc != ESP_OK)
    {
        return rc;
    }
    source = source >> 1;
    if (source & BIT(0))
    {
        rc = adxl345_enable_inactivity_int();
    }
    else
    {
        rc = adxl345_disable_inactivity_int();
    }
    adxl345_get_and_clear_int_source();
    return ESP_OK;
}

esp_err_t sm_tx_state_if_due(uint8_t flags)
{
    if (s_lora_event_group == NULL)
    {
        s_lora_event_group = xEventGroupCreate();
    }

    struct timeval now;
    gettimeofday(&now, NULL);
    long dt =
        (now.tv_sec - last_lora_tx_time.tv_sec) * 1000L +
        (now.tv_usec - last_lora_tx_time.tv_usec) / 1000L;

    if (dt <= LORA_COOLDOWN_S * 1000 && last_lora_tx_time.tv_sec != 0 && last_lora_tx_time.tv_usec != 0)
    {
        xEventGroupSetBits(s_lora_event_group, LORA_TX_READY_BIT);
        return ESP_OK;
    }

    lora_msg_t *msg = (lora_msg_t *)malloc(sizeof(lora_msg_t));

    ESP_LOGI(TAG, "Sending TX: Battery only");
    msg->buffer[0] = flags;
    msg->tx_length = 1;

    tx(msg);

    return ESP_OK;
}

esp_err_t sm_tx_location(void)
{
    if (s_lora_event_group == NULL)
    {
        s_lora_event_group = xEventGroupCreate();
    }

    lora_msg_t *msg = (lora_msg_t *)malloc(sizeof(lora_msg_t));
    msg->tx_length = TX_BUF_SIZE;

    get_location_msg(msg);

    ESP_LOGI(TAG, "Sending TX: Location + Battery");
    msg->buffer[TX_BUF_SIZE - 1] = 0x00 /*battery_get_state()*/;

    tx(msg);

    return ESP_OK;
}

esp_err_t sm_wait_tx_done(void)
{

    EventBits_t bits = xEventGroupWaitBits(s_lora_event_group, LORA_TX_READY_BIT | LORA_RX_READY_BIT | LORA_FAIL_BIT, false, false, 30 * 1000 / portTICK_PERIOD_MS);
    if (bits & (LORA_FAIL_BIT))
    {
        return ESP_FAIL;
    }
    if (bits & (LORA_TX_READY_BIT | LORA_RX_READY_BIT))
    {
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}

lora_status_t sm_get_lora_status(void)
{
    EventBits_t bits = xEventGroupGetBits(s_lora_event_group);
    if (bits & LORA_FAIL_BIT)
    {
        return LORA_FAIL;
    }
    if (bits & LORA_TX_READY_BIT)
    {
        return LORA_TX_READY;
    }
    if (bits & LORA_RX_READY_BIT)
    {
        return LORA_RX_READY;
    }
    return LORA_BUSY;
}

esp_err_t sm_get_rx_data(char *buf, size_t buf_size)
{
    if (buf == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (buf_size < RX_BUF_SIZE)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    taskENTER_CRITICAL(&spinlock);
    memcpy(buf, _rx_data, RX_BUF_SIZE);
    taskEXIT_CRITICAL(&spinlock);

    return ESP_OK;
}