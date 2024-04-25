#ifndef __LIDAR_H__
#define __LIDAR_H__

#define LIDAR_UART_NUM 				UART_NUM_2
#define LIDAR_MOTOR_CONTROL_PIN		5
#define LIDAR_UART_TX				17
#define LIDAR_UART_RX				16
#define LIDAR_DATA_PACKET_SIZE		5
#define LIDAR_UART_QUEUE_SIZE		200
#define BUF_SIZE					1024
#define RD_BUF_SIZE 				(BUF_SIZE)
typedef struct {
	float angle;
	float distant;
} Lidar_Data;

extern Lidar_Data lidar;

void lidar_init(void);
void lidar_print();
#endif