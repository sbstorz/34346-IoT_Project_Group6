#ifndef MRN2483_H
#define MRN2483_H

#include "uUART.h"
#include "driver/gpio.h"

/**
 * @brief Initializes the RN2483 LoRaWAN module.
 * This function configures the reset pin as output and initializes the UART port for communication.
 * If requested the module is also reset. A RX buffer is allocated.
 * @param uart_num the UART port the device should use
 * @param tx_io_num the TX GPIO pin on the board connected to the modules RX pin
 * @param rx_io_num the RX GPIO pin on the board connected to the modules TX pin
 * @param rst_io_num the reset GPIO pin on the board connected to the modules reset pin
 * @param rx_buffer_size size of the receiver buffer in Bytes
 * @param reset if true the module is reset when calling this function. This should only be used
 * on initial boot since after a reset a network rejoin is required.
 * @return 0
 */
int rn_init(uart_port_t uart_num, gpio_num_t tx_io_num, gpio_num_t rx_io_num, gpio_num_t rst_io_num, int rx_buffer_size, bool reset);

/**
 * @brief resets the LoRaWAN module by pulling the reset pin low for 10 ms.
 */
void rn_reset(void);

/**
 * @brief Initiates the Over-the-air-activation joining procedure. Credentials are configured in KConfig.
 * If joining fails it is retried 3 times.
 * @return ESP_OK on success; ESP_FAIL on failure.
 */
esp_err_t rn_init_otaa(void);

/**
 * @brief Transmits a unconfirmed LoRaWAN message.
 * This function sends the TX command to the LoRaWAN module and parses the response. The most common
 * errors are handled internally. If the device is not joined the OTAA is initiated. For other failures it retries
 * up to 10 times, in the worst case resetting the module. After the transmission the response after the downlink window has
 * closed is awaited and handled. If a downlink is received it is decoded and returned.
 * @param tx_data a char buffer to transmit
 * @param tx_data_size if non-zero sets the number of bytes transmitted from the buffer.
 * If set to 0, strlen(tx_data) is used. This will cause problems when transmitting non-ASCII data containing zeros.
 * @param tx_port the port the message is sent to.
 * @param encode set to false if the buffer already contains ASCII characters of the bytes to transmit.
 * should usually be set to true. If one wants to transmit {0x0a, 0x10} then the LoRaWAN module expects the message to be sent as:
 * "0A10". Set encode to true to automatically decode the char buffer in this way.
 * @param rx_data a allocated buffer to store the received message.
 * @param rx_data_size Size of the allocated RX buffer, needs to be big enough to hold the received payload. Currently no
 * checks are performed
 * @param rx_port reference to store the port the received message was sent to.
 * @return ESP_OK on success, ESP_FAIL if failed to send message or failed to decode payload.
 */
esp_err_t rn_tx(char *tx_data, size_t tx_data_size, unsigned int tx_port, bool encode, char *rx_data, size_t rx_data_size, unsigned int *rx_port);

/**
 * @brief Sends the autobaud sequence.
 * This can be used to set a new baud-rate but more importantly wakes the module from sleep.
 * The mechanism is to send a BREAK signal on the TX line to the module. For this the
 * Modules RX pin is pulled low for 100 ms, which is longer than the tranmission duration of
 * a normal character at the current baud-rate. Subsequently a single, unterminated 0x55 character
 * is sent. To check if communication is established again a command with known response (version) is sent.
 *
 * @return ESP_OK on success, ESP_FAIL the communication check failed.
 */
esp_err_t rn_set_autobaud(void);

/**
 * @brief Puts the LoRaWAN module to sleep.

 * @return ESP_OK on success, ESP_FAIL if we got a response to the sleep command, indicating the
* device is in fact not sleeping.
 */
esp_err_t rn_sleep(void);

/**
 * @brief Wakes the LoRaWAN module.
 * Tries up to 5 times to wake the device by sending the autobaud signal
 *
 * @return ESP_OK on success, ESP_FAIL if all attempts have failed.
 */
esp_err_t rn_wake(void);

#endif // MRN2483_H