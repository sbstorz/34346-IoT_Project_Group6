#ifndef UUART_H
#define UUART_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"


typedef enum
{
    CRLF = 1,
} delimiter_t;

/**
 * @brief initialize a UART driver with 8N1 protocol.
  *
 * @param uart_num UART Port numberto initialize.
 * @param tx_io_num TX pin tp use.
 * @param rx_io_num RX pin tp use.
 * @param baudrate the baudrate to use.
 * @param rx_buffer_size the hardware RX buffer size. Should be large enough to
 * hold multiple messages without overflow
 */
void uart_init_driver(uart_port_t uart_num, int tx_io_num, int rx_io_num, int baudrate, int rx_buffer_size);

/**
 * @brief read UART buffer up to a delimiter to application. This reads data from the UART RX buffer
 * bytewise and checks if the specified delimiter has been reached.
 *
 * @param uart_num UART Port number
 * @param delimiter Currenlty only `CRLF` supported
 * @param rxBuf Must be big enough to hold entire line inclusive the delimiter.
 * @param buf_size Size of the buffer to avoid writting outside it.
 * @param timeout_ms timeout for intercharacter reception.
 *
 * @return error code, or number of read bytes up to delimiter
 */
int uart_read_data_to_delimiter(uart_port_t uart_num, delimiter_t delimiter, char *rxBuf, size_t buf_size, unsigned int timeout_ms);

#endif // UUART_H
