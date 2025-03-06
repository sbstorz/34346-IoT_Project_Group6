#ifndef UHTTPSERVER_H
#define UHTTPSERVER_H

#include <esp_http_server.h>
#include <sys/param.h>
#include "esp_system.h"
#include "esp_check.h"
#include <esp_wifi.h>


#define HTTP_QUERY_KEY_MAX_LEN  (64)

httpd_handle_t http_start_server(void);
esp_err_t http_stop_server(httpd_handle_t server);

extern httpd_handle_t server;

#endif // UHTTPSERVER_H