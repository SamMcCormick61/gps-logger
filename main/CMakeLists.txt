idf_component_register(SRCS "main.c" "gps.c" "logger.c" "status_led.c" "webui.c" "motion.c" "power.c"
                       INCLUDE_DIRS "."
                       EMBED_TXTFILES "web/index.html"
                       REQUIRES esp_driver_uart app_update esp_timer esp_driver_rmt
                                esp_driver_gpio esp_driver_i2c esp_wifi esp_netif esp_event nvs_flash
                                esp_http_server espressif__mdns)
