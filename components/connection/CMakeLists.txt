idf_component_register(SRCS "socket.c" "wifi.c"
                    INCLUDE_DIRS "include" 
                    REQUIRES esp_wifi nvs_flash uart)
