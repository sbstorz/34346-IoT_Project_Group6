#ifndef UI2C_H_
#define UI2C_H_

#include "driver/i2c_master.h"

// #define SCL_IO_PIN 36
// #define SDA_IO_PIN 33

#define SCL_IO_PIN 22
#define SDA_IO_PIN 21

#define PORT_NUMBER -1

void i2c_init_master(void);
esp_err_t i2c_add_slave(uint16_t address, i2c_master_dev_handle_t *dev_handle);

#endif // UI2C_H_
