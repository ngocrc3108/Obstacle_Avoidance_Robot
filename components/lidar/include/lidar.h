#ifndef __LIDAR_H__
#define __LIDAR_H__

#include <stdint.h>
#include <stddef.h>

#define LIDAR_DATA_PACKET_SIZE		5

typedef struct {
	float angle;
	float distant;
} Lidar_Data;

void lidar_init(void);
void lidar_print(Lidar_Data point, char* pointName);
void lidar_handler(uint8_t* raw_data);
void lidar_start();

#endif