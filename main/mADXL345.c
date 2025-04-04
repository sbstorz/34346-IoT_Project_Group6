#include "mADXL345.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"

#define TAG "ADXL345"

void adxl345_init(uint8_t i2cAddress) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2cAddress,
        .scl_speed_hz = 100000,
    };
}
