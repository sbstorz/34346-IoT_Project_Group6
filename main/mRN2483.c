#include "mRN2483.h"

#define TAG "RN2483"

static gpio_num_t RST_PIN;
static gpio_num_t RX_PIN;
static gpio_num_t TX_PIN;
static uart_port_t UART_PORT;

static int BAUDRATE = 57600;
static int RX_BUF_SIZE = 1024;
char *rxBuf;

/**
 * @brief enocdes a char buffer to a HEX string representation
 * 
 * @warning returned buffer must be freed manually!
 * @param dst allocated buffer of size `strlen(src) * 2 + 1` or bigger
 * @param src `\0` delimited string buffer
 */
static void base16encode(const char *dst, const char *src)
{
    // char *data_buf = (char *)malloc(strlen(src) * 2 + 1); // Multiply with 2 since every char needs to be encoded as a HEX value with two chars.
    for (size_t i = 0; i < strlen(src); i++)
    {
        sprintf(&dst[i * 2], "%02x", src[i]);
    }
}

/**
 * @brief Decodes a HEX string representation to a char buffer
 * @warning Returned buffer must be freed manually!
 * 
 * @param dst allocated buffer of size `strlen(src) / 2 + 1` or bigger
 * @param src `\0` delimited string buffer
 *
 */
static void base16decode(char *dst, const char *src)
{
    size_t len = strlen(src);
    
    for (size_t i = 0; i < len; i += 2) {
        // Convert two hex characters to a byte
        char hex_byte[3] = { src[i], src[i + 1], '\0' }; // Null-terminate the string
        dst[i / 2] = (char)strtoul(hex_byte, NULL, 16);
    }

    dst[len / 2] = '\0'; // Null-terminate the decoded string
}

void rn_reset(void)
{

    gpio_set_level(RST_PIN, 0);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    gpio_set_level(RST_PIN, 1);
}

int rn_init(uart_port_t uart_num, gpio_num_t tx_io_num, gpio_num_t rx_io_num, gpio_num_t rst_io_num, int rx_buffer_size)
{
    RST_PIN = rst_io_num;
    TX_PIN = tx_io_num;
    RX_PIN = rx_io_num;
    UART_PORT = uart_num;
    RX_BUF_SIZE = rx_buffer_size;

    gpio_reset_pin(RST_PIN);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(RST_PIN, GPIO_MODE_OUTPUT);

    uart_init_driver(UART_PORT, TX_PIN, RX_PIN, BAUDRATE, RX_BUF_SIZE);
    uart_flush(UART_PORT);

    rn_reset();

    rxBuf = (char *)malloc(RX_BUF_SIZE * sizeof(char));
    memset(rxBuf, 0, RX_BUF_SIZE);
    uart_read_data_to_delimiter(UART_PORT, CRLF, rxBuf, RX_BUF_SIZE, 500);
    // uart_flush(UART_PORT);

    return 0;
}

esp_err_t rn_init_otaa(void)
{

    ESP_LOGI(TAG, "mac reset");
    char *rc = rn_send_raw_cmd("mac reset 868");
    if (rc == NULL || strcmp(rc, "ok") != 0)
    {
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "mac set deveui");
    rc = rn_send_raw_cmd("mac set deveui 0004A30B010D3F45");
    if (rc == NULL || strcmp(rc, "ok") != 0)
    {
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "mac set appeui");
    rc = rn_send_raw_cmd("mac set appeui BE7A000000001465");
    if (rc == NULL || strcmp(rc, "ok") != 0)
    {
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "mac set appkey");
    rc = rn_send_raw_cmd("mac set appkey A5FFC8F7C29D6EE92CC1141775C46162");
    if (rc == NULL || strcmp(rc, "ok") != 0)
    {
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "radio set pwr");
    rc = rn_send_raw_cmd("radio set pwr 14");
    if (rc == NULL || strcmp(rc, "ok") != 0)
    {
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "mac set dr");
    rc = rn_send_raw_cmd("mac set dr 2");
    if (rc == NULL || strcmp(rc, "ok") != 0)
    {
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "mac set adr");
    rc = rn_send_raw_cmd("mac set adr off");
    if (rc == NULL || strcmp(rc, "ok") != 0)
    {
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "mac set ar");
    rc = rn_send_raw_cmd("mac set ar off");
    if (rc == NULL || strcmp(rc, "ok") != 0)
    {
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "mac join");
    rc = rn_send_raw_cmd("mac join otaa");
    if (rc == NULL || strcmp(rc, "ok") != 0)
    {
        return ESP_FAIL;
    }

    int n = uart_read_data_to_delimiter(UART_PORT, CRLF, rxBuf, RX_BUF_SIZE, 10000);
    if (n <= 0)
    {
        return ESP_FAIL;
    }

    rxBuf[n] = 0;
    if (strcmp(rxBuf, "accepted") != 0)
    {
        return ESP_FAIL;
    }

    return ESP_OK;
}

// TODO: should be static and is not thread safe!
char *rn_send_raw_cmd(const char *cmd)
{
    const char del[] = {'\r', '\n'};
    uart_flush(UART_PORT);
    int n = uart_write_bytes(UART_PORT, cmd, strlen(cmd));
    n += uart_write_bytes(UART_PORT, del, 2);

    if (n != strlen(cmd) + 2)
    {
        return NULL;
    }
    n = uart_read_data_to_delimiter(UART_PORT, CRLF, rxBuf, RX_BUF_SIZE, 500);
    if (n <= 0)
    {
        return NULL;
    }

    rxBuf[n] = 0;
    return rxBuf;
}

esp_err_t rn_set_autobaud(void)
{
    uart_set_line_inverse(UART_PORT, UART_SIGNAL_TXD_INV);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    uart_set_line_inverse(UART_PORT, UART_SIGNAL_INV_DISABLE);
    char autobaud_string[3] = {0x55, '\r', '\n'};
    int f_retval = uart_write_bytes(UART_PORT, autobaud_string, 1);

    // CHECK if comm established

    return ESP_OK;
}

esp_err_t rn_wake(void)
{
    return rn_set_autobaud();
}

esp_err_t rn_tx(char *tx_data, unsigned int tx_port, bool encode, char *rx_data, size_t rx_data_size, unsigned int *rx_port)
{
    *rx_port = 0;
    /* Send the tx command with unconfirmed mode*/
    const char *cmd = "mac tx uncnf ";
    uart_flush(UART_PORT);
    if (uart_write_bytes(UART_PORT, cmd, strlen(cmd)) < strlen(cmd))
    {
        return ESP_ERR_NOT_FINISHED;
    }

    /* Transform the integer tx_port to string and send it*/
    const char buf[5]; // max 123 as tx_port -> three chars + NULL + space
    sprintf(buf, "%d ", tx_port);
    if (uart_write_bytes(UART_PORT, buf, strlen(buf)) < strlen(buf))
    {
        return ESP_ERR_NOT_FINISHED;
    }

    if (encode)
    {
        char *data_buf = (char *)malloc(strlen(tx_data) * 2 + 1); // Multiply with 2 since every char needs to be encoded as a HEX value with two chars.
        base16encode(data_buf, tx_data);
        if (uart_write_bytes(UART_PORT, data_buf, strlen(data_buf)) < strlen(data_buf))
        {
            return ESP_ERR_NOT_FINISHED;
        }
        free(data_buf);
    }
    else
    {
        if (uart_write_bytes(UART_PORT, tx_data, strlen(tx_data)) < strlen(tx_data))
        {
            return ESP_ERR_NOT_FINISHED;
        }
    }

    ESP_LOGI(TAG, "mac tx");
    char *rc = rn_send_raw_cmd("");
    if (strcmp(rc, "ok") != 0)
    {
        return ESP_FAIL;
    }

    int n = uart_read_data_to_delimiter(UART_PORT, CRLF, rxBuf, RX_BUF_SIZE, 10000);
    if (n <= 0)
    {
        return ESP_FAIL;
    }

    rxBuf[n] = 0;
    if (strcmp(rxBuf, "mac_tx_ok") == 0)
    {
        return ESP_OK;
    }
    if (strncmp(rxBuf, "mac_rx ", 7) == 0)
    {
        char *buf = (char*)malloc(rx_data_size*2);
        // Use sscanf to extract port number and data
        int result = sscanf(rxBuf + 7, "%u %s", rx_port, buf);

        // Check if the extraction was successful
        if (result != 2)
        {
            ESP_LOGE(TAG,"Failed to extract port number and data.");
            free(buf);
            return ESP_FAIL;
        }

        // Validate port number
        if (*rx_port < 1 || *rx_port > 223)
        {
            ESP_LOGE(TAG,"Port number out of range: %d", *rx_port);
            free(buf);
            return 1;
        }

        base16decode(rx_data, buf);
        free(buf);

        return ESP_OK;
    }

    return ESP_OK;
}