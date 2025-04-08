#include "uUART.h"

static const char *TAG = "UART";


int uart_read_data_to_delimiter(uart_port_t uart_num, delimiter_t delimiter, char *rxBuf, size_t buf_size, unsigned int timeout_ms)
{
    char del_char;

    if (rxBuf == NULL)
    {
        ESP_LOGE(TAG, "Invalid RX Buffer");
        return -1;
    }
    if (buf_size < 1)
    {
        ESP_LOGE(TAG, "buf_size must be greater than 0");
        return -2;
    }

    switch (delimiter)
    {
    case CRLF:
        del_char = '\n';
        break;

    default:
        ESP_LOGE(TAG, "Delimiter not supported yet");
        return -4;
        break;
    }

    int length = 0;
    int totLength = 0;
    do
    {
        if (totLength >= buf_size)
        {
            ESP_LOGE(TAG, "Insufficient buffer space");
            return -3;
        }

        length = uart_read_bytes(uart_num, &rxBuf[totLength], 1, timeout_ms / portTICK_PERIOD_MS);
        
        if (rxBuf[totLength] == del_char)
        {
            break;
        }
        totLength += length;

    } while (length > 0);
    if (totLength == 0)
    {
        ESP_LOGW(TAG, "No Data received");
        return 0;
    }
    if (length < 0)
    {
        ESP_LOGE(TAG, "Error receiving data");
        return -1;
    }
    ESP_LOGI(TAG, "Data received");
    ESP_LOG_BUFFER_HEXDUMP(TAG, rxBuf, totLength, ESP_LOG_INFO);

    switch (delimiter)
    {
    case CRLF:
        totLength -= 1;
        totLength = totLength < 0 ? 0 : totLength;
        break;

    default:
        break;
    }

    return totLength;
}

void uart_init_driver(uart_port_t uart_num, int tx_io_num, int rx_io_num, int baudrate, int rx_buffer_size)
{
    const uart_config_t uart_config = {
        .baud_rate = baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(uart_num, rx_buffer_size, 0, 0, NULL, 0);
    uart_param_config(uart_num, &uart_config);
    uart_set_pin(uart_num, tx_io_num, rx_io_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}