#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#include "uart.h"
#include "car.h"
#include "lidar.h"
#include "connection/socket.h"
#include "connection/wifi.h"
#include "led.h"

void app_main(void) {        
    led_init();

    uart_init();
    uart_enable_interrupt();

    wifi_init_sta();
    socket_start_send_async();
    tcp_server_init();
    esp_err_t ret = socket_start_listen();

    if(ret == ESP_OK) {
        lidar_init();
        car_init();
    }
    else {
        ESP_LOGI("DEBUG", "socket fail");
    }
}