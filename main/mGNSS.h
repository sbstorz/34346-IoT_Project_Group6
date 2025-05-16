#ifndef MGNSS_H
#define MGNSS_H

#include "driver/gpio.h"
#include "driver/uart.h"


int test();
esp_err_t gnss_init(uart_port_t uart_num, gpio_num_t tx_io_num, gpio_num_t rx_io_num, gpio_num_t pwr_io_num);
esp_err_t gnss_deinit(void);
esp_err_t gnss_get_location(int32_t *latitudeX1e7,int32_t *longitudeX1e7,int32_t *hMSL,int32_t *hAcc,int32_t *vAcc, unsigned int timeout_s, bool allowSleep);
esp_err_t gnss_set_eco_mode(void);
esp_err_t gnss_sleep(void);
esp_err_t gnss_wake(void);
#endif // MGNSS_H