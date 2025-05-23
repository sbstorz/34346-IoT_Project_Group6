#include "mRN2483.h"

#define TAG "RN2483"

typedef enum
{
    busy,
    frame_counter_err_rejoin_needed,
    invalid_data_len,
    invalid_param,
    mac_err,
    mac_paused,
    mac_rx,
    mac_tx_ok,
    no_free_ch,
    not_joined,
    ok,
    radio_err,
    radio_tx_ok,
    silent,
    EMPTY,
    UNKNOWN
} received_t;

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
static void base16encode(char *dst, const char *src, size_t src_size)
{
    size_t len = src_size ? src_size : strlen(src);
    for (size_t i = 0; i < len; i++)
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
static void base16decode(char *dst, const char *src, size_t src_size)
{
    size_t len = src_size ? src_size : strlen(src);

    for (size_t i = 0; i < len; i += 2)
    {
        // Convert two hex characters to a byte
        char hex_byte[3] = {src[i], src[i + 1], '\0'}; // Null-terminate the substring
        dst[i / 2] = (char)strtoul(hex_byte, NULL, 16);
    }

    dst[len / 2] = '\0'; // Null-terminate the decoded string
}

/**
 * @brief parses a response buffer and returns the correponding enumerator
 *
 * @param rsp `\0` delimited string buffer
 * @return `received_t` object
 *
 */
static received_t parse_response(const char *rsp)
{
    if (rsp == NULL)
    {
        return EMPTY;
    }
    if (strcmp(rsp, "ok") == 0)
    {
        return ok;
    }
    else if (strcmp(rsp, "busy") == 0)
    {
        return busy;
    }
    else if (strcmp(rsp, "frame_counter_err_rejoin_needed") == 0)
    {
        return frame_counter_err_rejoin_needed;
    }
    else if (strcmp(rsp, "invalid_data_len") == 0)
    {
        return invalid_data_len;
    }
    else if (strcmp(rsp, "invalid_param") == 0)
    {
        return invalid_param;
    }
    else if (strcmp(rsp, "mac_err") == 0)
    {
        return mac_err;
    }
    else if (strcmp(rsp, "mac_paused") == 0)
    {
        return mac_paused;
    }
    else if (strcmp(rsp, "mac_rx") == 0)
    {
        return mac_rx;
    }
    else if (strcmp(rsp, "mac_tx_ok") == 0)
    {
        return mac_tx_ok;
    }
    else if (strcmp(rsp, "no_free_ch") == 0)
    {
        return no_free_ch;
    }
    else if (strcmp(rsp, "not_joined") == 0)
    {
        return not_joined;
    }
    else if (strcmp(rsp, "radio_err") == 0)
    {
        return radio_err;
    }
    else if (strcmp(rsp, "radio_tx_ok") == 0)
    {
        return radio_tx_ok;
    }
    else if (strcmp(rsp, "silent") == 0)
    {
        return silent;
    }
    else
    {
        return UNKNOWN;
    }
}

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

/**
 * @brief sends a mac tx command with data and returns the immediate response
 *
 * @param tx_data `\0` delimited string buffer
 * @param tx_port port number 1 - 223
 * @param encode if the payload is to be interpreted as a string buffer
 * @param rx_data pointer to buffer holding the immediate response data (points to global rx_buf)
 * @return esp error type
 *
 */
static esp_err_t send_tx_cmd(char *tx_data, size_t tx_data_size, unsigned int tx_port, bool encode, char **rx_data)
{
    ESP_LOGI(TAG, "mac tx");
    /* Send the tx command with unconfirmed mode*/
    const char *cmd = "mac tx uncnf ";
    uart_flush(UART_PORT);
    if (uart_write_bytes(UART_PORT, cmd, strlen(cmd)) < strlen(cmd))
    {
        return ESP_ERR_NOT_FINISHED;
    }

    /* Transform the integer tx_port to string and send it*/
    char buf[5]; // max 123 as tx_port -> three chars + NULL + space
    sprintf(buf, "%d ", tx_port);
    if (uart_write_bytes(UART_PORT, buf, strlen(buf)) < strlen(buf))
    {
        return ESP_ERR_NOT_FINISHED;
    }

    size_t len = tx_data_size ? tx_data_size : strlen(tx_data);
    if (encode)
    {

        char *data_buf = (char *)malloc(len * 2 + 1); // Multiply with 2 since every char needs to be encoded as a HEX value with two chars.
        base16encode(data_buf, tx_data, len);
        if (uart_write_bytes(UART_PORT, data_buf, strlen(data_buf)) < strlen(data_buf))
        {
            return ESP_ERR_NOT_FINISHED;
        }

        free(data_buf);
    }
    else
    {
        if (uart_write_bytes(UART_PORT, tx_data, len) < len)
        {
            return ESP_ERR_NOT_FINISHED;
        }
    }

    *rx_data = rn_send_raw_cmd("");
    return ESP_OK;
}

void rn_reset(void)
{
    gpio_set_level(RST_PIN, 0);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    gpio_set_level(RST_PIN, 1);
}

int rn_init(uart_port_t uart_num, gpio_num_t tx_io_num, gpio_num_t rx_io_num, gpio_num_t rst_io_num, int rx_buffer_size, bool reset)
{
    RST_PIN = rst_io_num;
    TX_PIN = tx_io_num;
    RX_PIN = rx_io_num;
    UART_PORT = uart_num;
    RX_BUF_SIZE = rx_buffer_size;

    /* Set the GPIO as a push/pull output */
    gpio_config_t cfg = {
        .pin_bit_mask = BIT64(RST_PIN),
        .mode = GPIO_MODE_OUTPUT_OD,
        .pull_up_en = false,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);

    uart_init_driver(UART_PORT, TX_PIN, RX_PIN, BAUDRATE, RX_BUF_SIZE);
    uart_flush(UART_PORT);

    if (reset)
    {
        rn_reset();
        vTaskDelay(500/portTICK_PERIOD_MS);
    }

    rxBuf = (char *)malloc(RX_BUF_SIZE * sizeof(char));
    memset(rxBuf, 0, RX_BUF_SIZE);

    return 0;
}

esp_err_t rn_init_otaa(void)
{
    const int retry_cnt = 3;

    ESP_LOGI(TAG, "mac reset");
    char *rc = rn_send_raw_cmd("mac reset 868");
    if (rc == NULL || strcmp(rc, "ok") != 0)
    {
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "mac set deveui");
    rc = rn_send_raw_cmd("mac set deveui " CONFIG_LORAWAN_DEVEUI);
    if (rc == NULL || strcmp(rc, "ok") != 0)
    {
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "mac set appeui");
    rc = rn_send_raw_cmd("mac set appeui " CONFIG_LORAWAN_APPEUI);
    if (rc == NULL || strcmp(rc, "ok") != 0)
    {
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "mac set appkey");
    rc = rn_send_raw_cmd("mac set appkey " CONFIG_LORAWAN_APPKEY);
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
    rc = rn_send_raw_cmd("mac set dr 3");
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

    for (size_t i = 0; i < retry_cnt; i++)
    {
        ESP_LOGI(TAG, "mac join");
        rc = rn_send_raw_cmd("mac join otaa");
        if (rc == NULL || strcmp(rc, "ok") != 0)
        {
            return ESP_FAIL;
        }

        int n = uart_read_data_to_delimiter(UART_PORT, CRLF, rxBuf, RX_BUF_SIZE, 20000);
        if (n <= 0)
        {
            return ESP_FAIL;
        }

        rxBuf[n] = 0;

        if (strcmp(rxBuf, "accepted") == 0)
        {
            break;
            ;
        }
    }

    return ESP_OK;
}

esp_err_t rn_set_autobaud(void)
{
    uart_set_line_inverse(UART_PORT, UART_SIGNAL_TXD_INV);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    uart_set_line_inverse(UART_PORT, UART_SIGNAL_INV_DISABLE);
    char autobaud_string[1] = {0x55};
    uart_write_bytes(UART_PORT, autobaud_string, 1);

    /* Check if back alive*/
    char *rc = rn_send_raw_cmd("sys get ver");
    if (rc == NULL || strncmp(rc, "RN2483", 6) != 0)
    {
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t rn_sleep(void)
{
    char *rc = rn_send_raw_cmd("sys sleep 4294967290");

    /* On success we do not get a response*/
    if (rc != NULL)
    {
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t rn_wake(void)
{
    unsigned int i = 0;
    while (1)
    {
        if (rn_set_autobaud() == ESP_OK)
        {
            return ESP_OK;
        }
        if (i++ >= 5)
        {
            return ESP_FAIL;
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    return ESP_FAIL;
}

esp_err_t rn_tx(char *tx_data, size_t tx_data_size, unsigned int tx_port, bool encode, char *rx_data, size_t rx_data_size, unsigned int *rx_port)
{
    *rx_port = 0;

    bool stop = 0;
    unsigned int i = 0;
    do
    {
        char *rc;
        send_tx_cmd(tx_data, tx_data_size, tx_port, encode, &rc);
        switch (parse_response(rc))
        {
        case ok:
            stop = 1;
            break;

        case frame_counter_err_rejoin_needed:
        case not_joined:
            if (rn_init_otaa() != ESP_OK)
            {
                return ESP_FAIL;
            }
            break;

        case busy:
        case no_free_ch:
            vTaskDelay(10000 / portTICK_PERIOD_MS);
            break;
        case invalid_param:
            vTaskDelay(100 / portTICK_PERIOD_MS);
            break;
        default:
            rn_reset();
            uart_read_data_to_delimiter(UART_PORT, CRLF, rxBuf, RX_BUF_SIZE, 1000);
            break;
        }
        i++;
    } while (!stop && i < 10);

    if (i >= 10)
    {
        return ESP_FAIL;
    }

    int n = uart_read_data_to_delimiter(UART_PORT, CRLF, rxBuf, RX_BUF_SIZE, 20000);
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
        char *buf = (char *)malloc(rx_data_size * 2);
        // Use sscanf to extract port number and data
        int result = sscanf(rxBuf + 7, "%u %s", rx_port, buf);

        // Check if the extraction was successful
        if (result != 2)
        {
            ESP_LOGE(TAG, "Failed to extract port number and data.");
            free(buf);
            return ESP_FAIL;
        }

        // Validate port number
        if (*rx_port < 1 || *rx_port > 223)
        {
            ESP_LOGE(TAG, "Port number out of range: %d", *rx_port);
            free(buf);
            return ESP_FAIL;
        }

        base16decode(rx_data, buf, 0);
        free(buf);

        return ESP_OK;
    }

    return ESP_OK;
}