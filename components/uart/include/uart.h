#ifndef __UART_H__
#define __UART_H__

#include "lidar.h"
#include "freertos/FreeRTOS.h"

#define LIDAR_UART_NUM 				UART_NUM_2
#define LIDAR_MOTOR_CONTROL_PIN		18
#define LIDAR_UART_TX				17
#define LIDAR_UART_RX				16
#define LIDAR_UART_QUEUE_SIZE		100
#define BUF_SIZE					5*1024

extern QueueHandle_t uart_queue;

void uart_init();
void uart_enable_interrupt();

#endif