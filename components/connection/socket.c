/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "connection/socket.h"

#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "uart.h"
#include "driver/uart.h"
#include "hal/uart_hal.h"
#include "lidar.h"

#define PORT                        3333
#define KEEPALIVE_IDLE              5
#define KEEPALIVE_INTERVAL          5
#define KEEPALIVE_COUNT             3

typedef struct {
    size_t len;
    uint8_t data[1024];
} socket_send_async_packet_t;

static QueueHandle_t socket_send_async_queue;
static const char *TAG = "example";
static int sock;
int listen_sock;

static void print_hex(uint8_t* data, size_t len);
static void socket_listen_task(void* para);
static void socket_send_async_task(void* para);

static void print_hex(uint8_t* data, size_t len) {
    printf("{");
    for(int i = 0; i < len; i++) {
        if(i != 0)
            printf(", ");
        printf("0x%x", data[i]);
    }
    printf("};\n");
}

int socket_receive(uint8_t* data, size_t len) {
    char addr_str[128];
    int addr_family = AF_INET;
    int ip_protocol = 0;
    int keepAlive = 1;
    int keepIdle = KEEPALIVE_IDLE;
    int keepInterval = KEEPALIVE_INTERVAL;
    int keepCount = KEEPALIVE_COUNT;
    struct sockaddr_storage dest_addr;

    ssize_t recv_len = recv(sock, data, len - 1, 0);
    if (recv_len < 0) {
        ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
    } else if (recv_len == 0) {
        ESP_LOGW(TAG, "Connection closed");
        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t addr_len = sizeof(source_addr);
        sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            return 0;
        }

        // Set tcp keepalive option
        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
        // Convert ip address to string

        if (source_addr.ss_family == PF_INET) {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        }

        ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);        
    } else {
        data[recv_len] = 0; // Null-terminate whatever is received and treat it like a string
        //ESP_LOGI(TAG, "Received %d bytes.", len);
        //print_hex(data, recv_len);
    }

    return recv_len;    
}

void socket_send(uint8_t* data, size_t len) {
    // send() can return less bytes than supplied length.
    // Walk-around for robust implementation.
    int to_write = len;
    while (to_write > 0) {
        int written = send(sock, data + (len - to_write), to_write, 0);
        if (written < 0) {
            ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
            // Failed to retransmit, giving up
            return;
        }
        to_write -= written;
    }
}

static void socket_listen_task(void* para)
{
    ESP_LOGW("SOCKET", "running\n");
    int len = 0;
    uint8_t rx_buffer[128];
    do {
        len = socket_receive(rx_buffer, sizeof(rx_buffer));
        if(len == 2 && rx_buffer[0] == 0xA5 && rx_buffer[1] == 0x20) {
            lidar_start();
        } 
        else {
            uart_write_bytes(LIDAR_UART_NUM, rx_buffer, len);
            ESP_LOGI("DEBUG", "socket listen: %d bytes.", len);
            print_hex(rx_buffer, len);
        }
    } while(len >= 0);

    ESP_LOGE("DEBUG", "delete task");
    vTaskDelete(NULL);
}

void tcp_server_init() {
    char addr_str[128];
    int addr_family = AF_INET;
    int ip_protocol = 0;
    int keepAlive = 1;
    int keepIdle = KEEPALIVE_IDLE;
    int keepInterval = KEEPALIVE_INTERVAL;
    int keepCount = KEEPALIVE_COUNT;
    struct sockaddr_storage dest_addr;

    if (addr_family == AF_INET) {
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(PORT);
        ip_protocol = IPPROTO_IP;
    }

    listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        return;
    }
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    ESP_LOGI(TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        ESP_LOGE(TAG, "IPPROTO: %d", addr_family);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT);

    err = listen(listen_sock, 1);
    if (err != 0) {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }

    ESP_LOGI(TAG, "Socket listening");

    struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
    socklen_t addr_len = sizeof(source_addr);
    sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
        return;
    }

    // Set tcp keepalive option
    setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
    // Convert ip address to string

    if (source_addr.ss_family == PF_INET) {
        inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
    }

    ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);

    // shutdown(sock, 0);
    // close(sock);

CLEAN_UP:
    //close(listen_sock);
}

void socket_start_listen() {
    xTaskCreate(socket_listen_task, "socket_listen_task", 10*1024, NULL, 1, NULL);
}

void socket_send_async(uint8_t* data, size_t len) {
    //ESP_LOGI("DEBUG", "real len: %d", len);
    socket_send_async_packet_t packet;
    packet.len = len > sizeof(packet.data) ? sizeof(packet.data) : len;
    memcpy(packet.data, data, packet.len);
    xQueueSend(socket_send_async_queue, &packet, 0);
}

static void socket_send_async_task(void* para) {
    socket_send_async_packet_t packet;
    int count = 0;
    for(;;) {
        if(xQueueReceive(socket_send_async_queue, &packet, (TickType_t)portMAX_DELAY)) {
            // ESP_LOGI("DEBUG", "async len: %d. count: %d", packet.len, count++);
            // print_hex(packet.data, packet.len);
            socket_send(packet.data, packet.len);
        }
    }
}

void socket_start_send_async() {
    socket_send_async_queue = xQueueCreate(20, sizeof(socket_send_async_packet_t));
    if(socket_send_async_queue == NULL)
        ESP_LOGI("DEBUG", "can not create queue\n");
    else {
        ESP_LOGI("DEBUG", "create queue successfully\n");
    }
    
    xTaskCreate(socket_send_async_task, "socket_send_async_task", 10*1024, NULL, 10, NULL);    
}