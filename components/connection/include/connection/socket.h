#ifndef __SOCKET_H__
#define __SOCKET_H__

#include <stdint.h>
#include <stddef.h>
#include <esp_err.h>

int socket_receive(uint8_t* data, size_t len);
void socket_send(uint8_t* data, size_t len);
void socket_start_send_async();
void socket_send_async(uint8_t* data, size_t len);
void tcp_server_init();
esp_err_t socket_start_listen();
#endif