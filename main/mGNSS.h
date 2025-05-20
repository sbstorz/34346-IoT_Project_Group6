#ifndef MGNSS_H
#define MGNSS_H

#include "driver/gpio.h"
#include "driver/uart.h"

/**
 * @brief Initializes the GNSS module.
 * This function prepares the Ublox GNSS module for use, but does not
 * activate it. The power pin is used inverted since the module is connected 
 * behind a high-side switch.
 * @param uart_num the UART Port number the GNSS module should be attached to.
 * @param tx_io_num the GPIO number of the serial TX pin conntected to the modules RX pad.
 * @param rx_io_num the GPIO number of the serial RX pin conntected to the modules TX pad.
 * @param pwr_io_num the GPIO number of the pin that controls power to the module.
 * @return ESP_OK; ESP_FAIL if unable to open UART port.
 */
esp_err_t gnss_init(uart_port_t uart_num, gpio_num_t tx_io_num, gpio_num_t rx_io_num, gpio_num_t pwr_io_num);

/**
 * @brief Deinitializes the GNSS module.
 * @return ESP_OK.
 */
esp_err_t gnss_deinit(void);

/**
 * @brief Polls the GNSS module if it has a position fix.
 * The GNSS module must be turned on before this function is called. The GNSS module is asked for
 * its latest position. It continues to poll if no fix is available until the timeout has passed.
 * If a position is available 20 more positions are requested since the position tends to converge,
 * increasing horizontal accuracy.
 * @param latitudeX1e7 reference to store latitude result
 * @param longitudeX1e7 reference to store longitude result
 * @param hMSL reference to store height over mean sea level result
 * @param hAcc reference to store horizontal accuracy.
 * @param vAcc reference to store vertical accuracy.
 * @param timeout_s timeout in seconds after which the function returns when no fix has been found.
 * @param allowSleep set to true to go to light sleep for 10 s between polls to module if a position is available.
 * This should be set true for initial acquisition with a long timeout > 5 min to reduce power consumption.
 * @return ESP_OK if position acquired; ESP_FAIL if unable to acquire position within timeout.
 */
esp_err_t gnss_get_location(int32_t *latitudeX1e7,int32_t *longitudeX1e7,int32_t *hMSL,int32_t *hAcc,int32_t *vAcc, unsigned int timeout_s, bool allowSleep);

/**
 * @brief Turns the GNSS module off
 * @return ESP_OK on success, ESP_FAIL on failure.
 */
esp_err_t gnss_sleep(void);

/**
 * @brief Turns the GNSS module on
 * @return ESP_OK on success, ESP_FAIL on failure.
 */
esp_err_t gnss_wake(void);

#endif // MGNSS_H