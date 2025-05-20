#ifndef MDIGITALIO_H
#define MDIGITALIO_H

#include "esp_err.h"
#include "driver/gpio.h"

/**
 * @brief initializes the LED for use.
 * This function initializes the LED GPIO to prepare for use. It configures the GPIO as an output,
 * sets an initial state and disables the deep sleep hold.
 * @param pin GPIO pad number the LED is connected to.
 * @param level Initial level on initialization, 1: ON, 0: OFF
 * @return ESP_OK.
 */
esp_err_t led_init(gpio_num_t pin, const int32_t level);

/**
 * @brief turns the led on.
 * This function turns the led on by setting the output to 0.
 * @return ESP_OK; ESP_ERR_INVALID_STATE if pin not initialized.
 */
esp_err_t led_state_on(void);

/**
 * @brief turns the led off.
 * This function turns the led off by setting the output to 1.
 * @return ESP_OK; ESP_ERR_INVALID_STATE if pin not initialized.
 */
esp_err_t led_state_off(void);


/**
 * @brief Makes the LED blink with 10 Hz frequency.
 * This function makes the LED blink by attaching a callback to a hardware timer.
 * @return ESP_OK; ESP_ERR_INVALID_STATE if pin not initialized.
 */
esp_err_t led_start_blink(void);

/**
 * @brief Makes the LED stop blinking.
 * This function stops and deletes the timer to make the LED stop blinking.
 * @return ESP_OK; ESP_ERR_INVALID_STATE if the timer has not been started.
 */
esp_err_t led_stop_blink(void);

/**
 * @brief Checks if the LED blinking timer is already active.
 * @return 1 if true; 0 otherwise.
 */
int led_is_blinking(void);

/**
 * @brief Initializes the button for use.
 * This function initializes the button GPIO to prepare for use. It configures the GPIO as input,
 * enables the internal pull-down and attaches an ISR for the GPIO_INTR_ANYEDGE interrupt type.
 * and disables the deep sleep hold.
 * @param pin GPIO pad number the button is connected to.
 * @return ESP_OK.
 */
esp_err_t button_init(gpio_num_t pin);

/**
 * @brief Reads the GPIO level of the configured button.
 * @return 1 if high; 0 low or not configured.
 */
int button_get_level(void);

/**
 * @brief Checks if a rising edge on the button has occured.
 * When initalizing the button a ISR is attached to it which will set a Event flag on
 * rising and falling edges. This function returns the state of Rising edge flag.
 * 
 * @return 1 if a rising edge has occured since the last call; 0 if not.
 */
int button_had_rEdge(void);

/**
 * @brief Waits for a falling edge on the button.
 * Blocking function which waits for the button to be relased indicated by a falling edge
 * When initalizing the button a ISR is attached to it which will set a Event flag on
 * rising and falling edges.
 * 
 * @return ESP_OK; ESP_ERR_TIMEOUT on Timeout, which is portMAX_DELAY (0xffffffffUL ticks)
*/
esp_err_t button_wait_fEdge(void);

#endif // MDIGITALIO_H