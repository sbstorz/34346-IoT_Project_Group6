#include "mGNSS.h"
#include "ubxlib.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_sleep.h"

#define TAG "mGNSS"

static uGnssTransportHandle_t transportHandle;
static uDeviceHandle_t gnssHandle = NULL;

static esp_err_t get_location(int32_t *latitudeX1e7,
                              int32_t *longitudeX1e7,
                              int32_t *hMSL,
                              int32_t *hAcc,
                              int32_t *vAcc)
{
    char messageSend[128];
    int32_t messageSendSize;
    uGnssMessageId_t responseMessageId = {
        .type = U_GNSS_PROTOCOL_UBX,
        .id.ubx = ((uint16_t)0x01 << 8) | (uint16_t)0x02};
    char *pMsgReceive = NULL;

    // Populate messageSend and set messageSendSize here
    // Populate responseMessageId with the expected response here
    // gnssHandle is assumed to have already been set up
    messageSendSize = uUbxProtocolEncode(0x01, 0x02, NULL, 0, messageSend);
    uGnssMsgReceiveFlush(gnssHandle, false);
    if (uGnssMsgSend(gnssHandle, messageSend,
                     messageSendSize) == messageSendSize)
    {
        int32_t messageReceiveSize = uGnssMsgReceive(gnssHandle,
                                                     &responseMessageId,
                                                     &pMsgReceive, 0,
                                                     1000, NULL);
        if (messageReceiveSize >= 0)
        {

            // Process the received message here
            int32_t body_size = uUbxProtocolDecode(pMsgReceive, messageReceiveSize, NULL, NULL, pMsgReceive, messageReceiveSize, NULL);
            ESP_LOG_BUFFER_HEXDUMP(TAG, pMsgReceive, body_size, ESP_LOG_INFO);
            *longitudeX1e7 = uUbxProtocolUint32Decode(pMsgReceive + 4);
            *latitudeX1e7 = uUbxProtocolUint32Decode(pMsgReceive + 8);
            *hMSL = uUbxProtocolUint32Decode(pMsgReceive + 16);
            *hAcc = uUbxProtocolUint32Decode(pMsgReceive + 20);
            *vAcc = uUbxProtocolUint32Decode(pMsgReceive + 24);

            uPortFree(pMsgReceive);
        }
    }

    ESP_LOGI(TAG, "HACC: %ld", *hAcc);
    // check if fix, return error if not fixed;
    if (*hAcc <= 0 || *hAcc > 1000 * 1000)
    {
        return ESP_FAIL;
    }

    return ESP_OK;
}

// The entry point: before this is called the system
// clocks must have been started and the RTOS must be running;
// we are in task space.
esp_err_t gnss_init(uart_port_t uart_num, gpio_num_t tx_io_num, gpio_num_t rx_io_num, gpio_num_t pwr_io_num)
{

    // Initialise the APIs we will need
    uPortInit();
    uGnssInit();

    // Open a UART with the recommended buffer length
    // on your chosen UART HW block and on the pins
    // where the GNSS module's UART interface is
    // connected to your MCU: you need to know these
    // for your hardware, either set the #defines
    // appropriately or replace them with the right
    // numbers, using -1 for a pin that is not connected.
    transportHandle.uart = uPortUartOpen(uart_num,
                                         U_GNSS_UART_BAUD_RATE, NULL,
                                         U_GNSS_UART_BUFFER_LENGTH_BYTES,
                                         tx_io_num,
                                         rx_io_num,
                                         -1,
                                         -1);

    if (transportHandle.uart <= 0)
    {
        ESP_LOGE(TAG, "Failed UART OPEN");
        return ESP_FAIL;
    }

    // Add a GNSS instance, giving it the UART handle and
    // the pin that enables power to the GNSS module; use
    // -1 if there is no such pin.
    uGnssAdd(U_GNSS_MODULE_TYPE_M8,
             U_GNSS_TRANSPORT_UART, transportHandle,
             pwr_io_num | U_GNSS_PIN_INVERTED, false,
             &gnssHandle);

    // To get prints of the message exchange with the GNSS module
    uGnssSetUbxMessagePrint(gnssHandle, true);

    uGnssSetRetries(gnssHandle, 5);

    // int32_t bitmap = uGnssCfgGetProtocolOut(gnssHandle);
    // ESP_LOGI(TAG, "MSG RATE: %ld", bitmap);
    // if (bitmap & (1 << 1))
    // {
    //     uGnssCfgSetProtocolOut(gnssHandle, U_GNSS_PROTOCOL_NMEA, false);
    //     bitmap = uGnssCfgGetProtocolOut(gnssHandle);
    //     ESP_LOGI(TAG, "MSG RATE: %ld", bitmap);
    // }

    return ESP_OK;
}

esp_err_t gnss_deinit(void)
{
    // Calling these will also deallocate all the handles that
    // were allocated above.
    uGnssDeinit();
    uPortDeinit();
    return ESP_OK;
}

esp_err_t gnss_get_location(int32_t *latitudeX1e7,
                            int32_t *longitudeX1e7,
                            int32_t *hMSL,
                            int32_t *hAcc,
                            int32_t *vAcc,
                            unsigned int timeout_s,
                            bool allowSleep)
{
    if (allowSleep)
    {
        esp_sleep_enable_timer_wakeup(10 * 1000 * 1000);
    }
    int64_t start = esp_timer_get_time();
    int32_t best_latitudeX1e7, best_longitudeX1e7, best_hMSL, best_hAcc, best_vAcc;
    while (esp_timer_get_time() - start < timeout_s * 1000 * 1000)
    {
        if (get_location(&best_latitudeX1e7, &best_longitudeX1e7, &best_hMSL, &best_hAcc, &best_vAcc) == ESP_OK)
        {

            for (size_t i = 0; i < 20; i++)
            {
                get_location(latitudeX1e7, longitudeX1e7, hMSL, hAcc, vAcc);
                if (*hAcc < best_hAcc)
                {
                    best_latitudeX1e7 = *latitudeX1e7;
                    best_longitudeX1e7 = *longitudeX1e7;
                    best_hMSL = *hMSL;
                    best_hAcc = *hAcc;
                    best_vAcc = *vAcc;
                }
                vTaskDelay(500 / portTICK_PERIOD_MS);
            }

            *latitudeX1e7 = best_latitudeX1e7;
            *longitudeX1e7 = best_longitudeX1e7;
            *hMSL = best_hMSL;
            *hAcc = best_hAcc;
            *vAcc = best_vAcc;

            return ESP_OK;
        }
        if (allowSleep)
        {
            esp_light_sleep_start();
        }
        else
        {
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
    }

    *longitudeX1e7 = 0;
    *latitudeX1e7 = 0;
    *hMSL = 0;
    *hAcc = 0;
    *vAcc = 0;

    return ESP_FAIL;
}

esp_err_t gnss_set_eco_mode(void)
{
    const int payload_length = 2;
    char messageSend[payload_length + U_UBX_PROTOCOL_OVERHEAD_LENGTH_BYTES];
    int32_t messageSendSize;
    // uGnssMessageId_t responseMessageId = {
    //     .type = U_GNSS_PROTOCOL_UBX,
    //     .id.ubx = ((uint16_t)0x06 << 8) | (uint16_t)0x11};
    // // char *pMsgReceive = NULL;

    char payload[] = {8, 4};

    messageSendSize = uUbxProtocolEncode(0x06, 0x11, payload, payload_length, messageSend);
    if (uGnssMsgSend(gnssHandle, messageSend,
                     messageSendSize) == messageSendSize)
    {
        return ESP_OK;
    }
    return ESP_FAIL;
    {
        // int32_t messageReceiveSize = uGnssMsgReceive(gnssHandle,
        //                                              &responseMessageId,
        //                                              &pMsgReceive, 0,
        //                                              1000, NULL);
        // if (messageReceiveSize >= 0)
        // {

        //     // Process the received message here
        //     int32_t body_size = uUbxProtocolDecode(pMsgReceive, messageReceiveSize, NULL, NULL, pMsgReceive, messageReceiveSize, NULL);
        //     ESP_LOG_BUFFER_HEXDUMP(TAG, pMsgReceive, body_size, ESP_LOG_INFO);
        //     *longitudeX1e7 = uUbxProtocolUint32Decode(pMsgReceive + 4);
        //     *latitudeX1e7 = uUbxProtocolUint32Decode(pMsgReceive + 8);
        //     *hMSL = uUbxProtocolUint32Decode(pMsgReceive + 16);
        //     *hAcc = uUbxProtocolUint32Decode(pMsgReceive + 20);
        //     *vAcc = uUbxProtocolUint32Decode(pMsgReceive + 24);

        //     uPortFree(pMsgReceive);
        // }
    }
}

esp_err_t gnss_sleep(void)
{
    int32_t rc = uGnssPwrOffBackup(gnssHandle);
    if (rc != 0)
    {
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t gnss_wake(void)
{
    int32_t rc = uGnssPwrOn(gnssHandle);
    if (rc != 0)
    {
        return ESP_FAIL;
    }
    return ESP_OK;
}