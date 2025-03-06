#include <stdio.h>
#include "uwifi.h"
#include "uhttpServer.h"
#include "mLED.h"

static const char *TAG = "main";


void app_main(void)
{
    GPIO_configure_led();
    EventGroupHandle_t s_wifi_event_group = wifi_init_sta();
    http_start_server();
    GPIO_attach_ToggleHandler();

    xEventGroupWaitBits(s_wifi_event_group,WIFI_FAIL_BIT,pdFALSE,pdFALSE,portMAX_DELAY);
    
    ESP_LOGI(TAG, "Wifi Fail, Dying...");
    esp_restart();
}