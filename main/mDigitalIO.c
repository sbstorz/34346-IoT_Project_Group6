#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "mDigitalIO.h"

#define REDGE_BIT BIT0
#define FEDGE_BIT BIT1

static gpio_num_t _led_pin = GPIO_NUM_NC;
static gpio_num_t _button_pin = GPIO_NUM_NC;

static int _last_button_state = 1;

static esp_timer_handle_t periodic_timer;
static EventGroupHandle_t s_button_event_group = NULL;

static int led_state = 0;
static void periodic_timer_callback(void *arg)
{
    if (led_state)
    {
        led_state_off();
    }
    else
    {
        led_state_on();
    }
}

esp_err_t led_init(gpio_num_t pin, const int32_t level)
{
    _led_pin = pin;

    // gpio_reset_pin(RST_PIN);
    /* Set the GPIO as a push/pull output */
    gpio_config_t cfg = {
        .pin_bit_mask = BIT64(_led_pin),
        .mode = GPIO_MODE_OUTPUT,
        // for powersave reasons, the GPIO should not be floating, select pullup
        .pull_up_en = true,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);

    if (level)
    {
        led_state_on();
    }
    else
    {
        led_state_off();
    }

    gpio_hold_dis(_led_pin);

    return ESP_OK;
}

esp_err_t led_state_on(void)
{
    if (_led_pin == GPIO_NUM_NC)
    {
        return ESP_ERR_INVALID_STATE;
    }
    gpio_set_level(_led_pin, 0);
    led_state = 1;
    return ESP_OK;
}

esp_err_t led_state_off(void)
{
    if (_led_pin == GPIO_NUM_NC)
    {
        return ESP_ERR_INVALID_STATE;
    }
    gpio_set_level(_led_pin, 1);
    led_state = 0;
    return ESP_OK;
}

int led_is_blinking(void)
{
    return esp_timer_is_active(periodic_timer);
}

esp_err_t led_start_blink(void)
{
    if (_led_pin == GPIO_NUM_NC)
    {
        return ESP_ERR_INVALID_STATE;
    }

    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &periodic_timer_callback,
        /* name is optional, but may help identify the timer when debugging */
        .name = "led_blink"};

    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_start_periodic(periodic_timer, 200000));

    return ESP_OK;
}

esp_err_t led_stop_blink(void)
{
    if (esp_timer_stop(periodic_timer) != ESP_OK)
    {
        return ESP_ERR_INVALID_STATE;
    }
    if (esp_timer_delete(periodic_timer) != ESP_OK)
    {
        return ESP_ERR_INVALID_STATE;
    }

    return led_state_off();
}

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (gpio_get_level(_button_pin))
    {
        xEventGroupSetBitsFromISR(s_button_event_group, REDGE_BIT, &xHigherPriorityTaskWoken);
    }
    else
    {
        xEventGroupSetBitsFromISR(s_button_event_group, FEDGE_BIT, &xHigherPriorityTaskWoken);
    }
}

esp_err_t button_init(gpio_num_t pin)
{

    if (s_button_event_group == NULL)
    {
        s_button_event_group = xEventGroupCreate();
    }

    _button_pin = pin;

    /* Set the GPIO as a push/pull output */
    gpio_config_t cfg = {
        .pin_bit_mask = BIT64(_button_pin),
        .mode = GPIO_MODE_INPUT,
        // for powersave reasons, the GPIO should not be floating, select pullup
        .pull_up_en = false,
        .pull_down_en = true,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);

    gpio_set_intr_type(_button_pin, GPIO_INTR_ANYEDGE);

    // install gpio isr service
    gpio_install_isr_service(0);
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(_button_pin, gpio_isr_handler, NULL);

    return ESP_OK;
}

esp_err_t button_wait_fEdge(void)
{
    EventBits_t bits = xEventGroupWaitBits(s_button_event_group, FEDGE_BIT, true, false, portMAX_DELAY);
   
    if (bits & FEDGE_BIT)
    {
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}

int button_had_rEdge(void){
    if((xEventGroupClearBits(s_button_event_group, REDGE_BIT) & REDGE_BIT)/* || gpio_get_level(_button_pin)*/){
        return 1;
    }
    return 0;
}

int button_get_state(void)
{
    if (_button_pin == GPIO_NUM_NC)
    {
        return ESP_ERR_INVALID_STATE;
    }
    int r = 0;
    int state = gpio_get_level(_button_pin);
    if (!_last_button_state && state)
    {
        r = 1;
    }
    _last_button_state = state;
    return r;
}

int button_get_level(void){
    return gpio_get_level(_button_pin);
}