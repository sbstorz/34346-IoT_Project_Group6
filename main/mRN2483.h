#ifndef MRN2483_H
#define MRN2483_H

#include "uUART.h"
#include "driver/gpio.h"


typedef enum
{
    busy,
    frame_counter_err_rejoin_needed,
    invalid_data_len,
    invalid_param,
    mac_err,
    mac_paused,
    mac_rx,
    mac_tx_ok,
    no_free_ch,
    not_joined,
    ok,
    radio_err,
    radio_tx_ok,
    silent,
    EMPTY,
    UNKNOWN
} received_t;

void rn_reset(void);
int rn_init(uart_port_t uart_num, gpio_num_t tx_io_num, gpio_num_t rx_io_num, gpio_num_t rst_io_num, int rx_buffer_size);
char *rn_send_raw_cmd(const char *cmd);
esp_err_t rn_init_otaa(void);
esp_err_t rn_tx(char *tx_data, unsigned int tx_port, bool encode, char *rx_data, size_t rx_data_size, unsigned int *rx_port);
esp_err_t rn_set_autobaud(void);
esp_err_t rn_sleep(void);
esp_err_t rn_wake(void);


#endif // MRN2483_H