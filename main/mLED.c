#include "mLED.h"
#include "uhttpServer.h"

#define LED_PIN GPIO_NUM_2

static const char *TAG = "LED driver";

/* An HTTP POST handler */
static esp_err_t toggle_post_handler(httpd_req_t *req)
{
    // char buf[100];
    // int ret, remaining = req->content_len;

    // while (remaining > 0)
    // {
    //     /* Read the data for the request */
    //     if ((ret = httpd_req_recv(req, buf,
    //                               MIN(remaining, sizeof(buf)))) <= 0)
    //     {
    //         if (ret == HTTPD_SOCK_ERR_TIMEOUT)
    //         {
    //             /* Retry receiving if timeout occurred */
    //             continue;
    //         }
    //         return ESP_FAIL;
    //     }

    //     /* Send back the same data */
    //     httpd_resp_send_chunk(req, buf, ret);
    //     remaining -= ret;

    //     /* Log data received */
    //     ESP_LOGI(TAG, "=========== RECEIVED DATA ==========");
    //     ESP_LOGI(TAG, "%.*s", ret, buf);
    //     ESP_LOGI(TAG, "====================================");
    // }

    GPIO_toggle_led();

    // End response
    // Adding new header field to redirect to root
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_set_status(req, "303");
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static const httpd_uri_t toggle = {
    .uri = "/toggle",
    .method = HTTP_POST,
    .handler = toggle_post_handler,
    .user_ctx = NULL};

void GPIO_configure_led(void)
{
    gpio_reset_pin(LED_PIN);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(LED_PIN, GPIO_MODE_INPUT_OUTPUT);
}

void GPIO_attach_ToggleHandler(void)
{
    // check if server is running/handle not NULL and register uri handler
    if (server)
    {
        ESP_LOGI(TAG, "Registering LED toggle handler");
        ESP_ERROR_CHECK(httpd_register_uri_handler(server, &toggle));
    }
}

void GPIO_toggle_led(void)
{
    ESP_LOGI(TAG,"Toggle LED");
    gpio_set_level(LED_PIN, !gpio_get_level(LED_PIN));
}

