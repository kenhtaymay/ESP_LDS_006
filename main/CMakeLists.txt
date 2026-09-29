idf_component_register(SRCS "main.c" "lidar.c" "motor_pwm.c" "controller.c" "scan.c" "wifi.c" "web.c"
                       INCLUDE_DIRS "."
                       EMBED_TXTFILES "web/index.html"
                       REQUIRES esp_driver_uart esp_driver_mcpwm esp_driver_usb_serial_jtag esp_driver_gpio
                                nvs_flash esp_timer esp_wifi esp_netif esp_event esp_http_server
                                app_update esp_app_format)
