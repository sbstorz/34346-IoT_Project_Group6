#include <stdio.h>
#include <unistd.h> // needed for sleep
#include <string.h> // for memcpy
#include "esp_log.h"

#include "uI2C.h"
#include "mADXL345.h"

static const char *TAG = "main";

void app_main(void)
{
    // Initialize the I2C master
    i2c_init_master();

    // Add the ADXL345 as a slave device
    i2c_master_dev_handle_t adxl345_handle = NULL;
    if (i2c_add_slave(ADXL345_DEFAULT_ADDRESS, &adxl345_handle) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add ADXL345 as a slave device. Exiting.");
        return;
    }

    // Test communication with the ADXL345
    if (adxl345_chipid() == ESP_OK)
    {
        ESP_LOGI(TAG, "ADXL345 communication successful.");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to communicate with ADXL345. Exiting.");
        return;
    }

    // Initialize the ADXL345 sensor
    if (!adxl345_begin())
    {
        ESP_LOGE(TAG, "Failed to initialize ADXL345. Exiting.");
        return;
    }

    ESP_LOGI(TAG, "ADXL345 initialized successfully.");

    // Read and log acceleration data in a loop
    adxl345_xyz_t accel_data;
    while (1)
    {
        adxl345_get_accel(&accel_data);
        ESP_LOGI(TAG, "Acceleration Data - X: %.2f m/s^2, Y: %.2f m/s^2, Z: %.2f m/s^2",
                 accel_data.x_ms, accel_data.y_ms, accel_data.z_ms);
        sleep(1); // Delay for 1 second
    }
}
