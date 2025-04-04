#include "uI2C.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

static const char *TAG = "I2C Master";


i2c_master_bus_handle_t i2c_master_bus_handle = NULL;


void i2c_init_master(void) {
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = PORT_NUMBER,
        .scl_io_num = SCL_IO_PIN,
        .sda_io_num = SDA_IO_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &i2c_master_bus_handle));
}


esp_err_t i2c_add_slave(uint16_t address, i2c_master_dev_handle_t *dev_handle) {

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = 100000,
    };

    if (i2c_master_probe(i2c_master_bus_handle, address, 500) == ESP_OK) {
        return i2c_master_bus_add_device(i2c_master_bus_handle, &dev_cfg, dev_handle);
    }else{
        ESP_LOGE(TAG,"Could not add slave at adress %d",address);
        return ESP_ERR_NOT_FOUND;
    }
}
