#include <stdio.h>
#include <unistd.h> // needed for sleep
#include <string.h> // for memcpy
#include "esp_log.h"

#include "uI2C.h"
#include "mADXL345.h"

static const char *TAG = "main";

void app_main(void)
{
    i2c_init_master();
    i2c_master_dev_handle_t dev1_handle;
    if (i2c_add_slave(0x28, &dev1_handle) != ESP_OK) {
        return;
    }
    adxl345_init(ADXL345_DEFAULT_ADDRESS);

    uint8_t r_buf[sizeof(int)];
    int r_val;

    while (1)
    {
        ESP_ERROR_CHECK(i2c_master_receive(dev1_handle, r_buf, sizeof(int), 1000));
        memcpy(&r_val, r_buf, sizeof(int));
        ESP_LOGI(TAG, "Received: %d", r_val);
        sleep(1);
    }
}
