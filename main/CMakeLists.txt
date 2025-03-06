idf_component_register(SRCS "uhttpServer.c" "main.c" "uwifi.c" "mLED.c"
                    PRIV_REQUIRES esp_wifi nvs_flash esp_http_server esp_driver_gpio
                    INCLUDE_DIRS ".")