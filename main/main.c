#include <stdio.h>
#include <unistd.h> // needed for sleep
#include "mRN2483.h"

#define TAG "Main"

void app_main(void)
{
    rn_init(UART_NUM_2, GPIO_NUM_17, GPIO_NUM_16, GPIO_NUM_18, 1024, true);

    if (rn_init_otaa() != ESP_OK)
    {
        return;
    }

    ESP_LOGI(TAG, "Sending TX");
    unsigned int port;
    char rx_data[10];

    while (1)
    {
        if (rn_tx("Sunny Day!", 1, true, rx_data, 10, &port) == ESP_OK && port > 0)
        {
            ESP_LOGI(TAG, "Received RX, port: %d, data:", port);
            ESP_LOG_BUFFER_HEXDUMP(TAG, rx_data, strlen(rx_data), ESP_LOG_INFO);
        }
        ESP_LOGI(TAG, "VDD: %s", rn_send_raw_cmd("sys get vdd"));
        ESP_LOGI(TAG, "VER: %s", rn_send_raw_cmd("sys get ver"));
        ESP_ERROR_CHECK_WITHOUT_ABORT(rn_sleep());
        sleep(60);
        ESP_ERROR_CHECK_WITHOUT_ABORT(rn_wake());
    }

}