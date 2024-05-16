#include "lidar.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "math.h"

#include "uart.h"
#include "driver/uart.h"
#include "hal/uart_hal.h"

#include "car.h"

#define MAX_DISTANT 800

static void lidar_start();
static void lidar_reset_value(Lidar_Data* value);
static Lidar_Data lidar_convert_raw_data(uint8_t* raw_data);
static uint8_t lidar_is_raw_data_valid(uint8_t* raw_data);

static const char *TAG = "LIDAR";

Lidar_Data firstPointInNewScan;
uint32_t point_count;
Lidar_Data currentPoint;
Lidar_Data lastPoint;
Lidar_Data point_A;
Lidar_Data point_B;
int isDangerous = 0;

void lidar_handler(uint8_t* raw_data) {

    if((raw_data[0] & 0x03) == 0x01) {
        float turnAngle;

        //ESP_LOGI("DEBUG", "NEW SCAN");

        if(point_count == 0) {
            turnAngle = 0;
        } else if(point_count >= 2 && fabs(lastPoint.angle - firstPointInNewScan.angle) > fabs(point_B.angle - point_A.angle)) {
            point_A = lastPoint;
            point_B = firstPointInNewScan;

            turnAngle = (point_B.angle + (point_A.angle - 360)) / 2;
        } else {
            turnAngle = (point_B.angle + point_A.angle) / 2;
        }

        turnAngle = turnAngle > 180 ? turnAngle - 360 : turnAngle;
        //ESP_LOGI("DEBUG", "turn angle: %f", turnAngle);

        // lidar_print(point_A, "Point A");
        // lidar_print(point_B, "Point B");
        // ESP_LOGI("DEBUG", "point_count: %ld\n", point_count);

        point_count = 0;
        lidar_reset_value(&firstPointInNewScan);

        if (isDangerous) {
            ESP_LOGE("DEBUG", "turn angle: %f", turnAngle);
            if(fabs(turnAngle) < 10) {
                // be hon 10 do thi vua di chuyen vua xoay
                car_turn_by_angle_and_forward(turnAngle);
            }
            else {
                // lon hon 10 do thi dung yen, xoay tai cho
                car_turn_by_angle(turnAngle);
            }
        }
        else {
            ESP_LOGI("DEBUG", "turn angle: %f", turnAngle);
            car_go_forward();
        }
        
        lidar_reset_value(&point_A);
        lidar_reset_value(&point_B);
        isDangerous = 0;
    }
    
    if(!lidar_is_raw_data_valid(raw_data)) 
        return;

    lastPoint = currentPoint;
    currentPoint = lidar_convert_raw_data(raw_data);
    //lidar_print(currentPoint, "currentPoint");
    if ((currentPoint.angle < 20 || currentPoint.angle > 340) && currentPoint.distant < 500) {
        isDangerous = 1;
    }
    
    if(point_count == 0) {
        firstPointInNewScan = currentPoint;
        //lidar_print(firstPointInNewScan, "Fisrt Point");
    }

    if(currentPoint.angle - lastPoint.angle > point_B.angle - point_A.angle) {
        point_A = lastPoint;
        point_B = currentPoint;
    }

    point_count++;
}

void lidar_init(void) {
    esp_log_level_set(TAG, ESP_LOG_INFO);

    gpio_set_direction(LIDAR_MOTOR_CONTROL_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LIDAR_MOTOR_CONTROL_PIN, 0);

    uart_init();
    lidar_start();
    uart_enable_interrupt();

}

static void lidar_start() {
    uint8_t startCommand[] = {0xA5, 0x20};
	uint8_t expected_response[] = {0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81};
    uint8_t buffer[100] = {0, 0, 0, 0, 0, 0, 0, 0};

    int tryCount = 0;
    int isStartSuccess = 0;
    do {
        uart_flush_input(LIDAR_UART_NUM);
        int ret = uart_write_bytes(LIDAR_UART_NUM, startCommand, sizeof(startCommand));
        ESP_LOGI(TAG, "WRITE CMD LEN: %d", ret);
        ret = uart_read_bytes(LIDAR_UART_NUM, buffer, 7, 5000 / portTICK_PERIOD_MS);
        ESP_LOGE(TAG, "Try start count: %d", tryCount++);
        ESP_LOGI(TAG, "READ RESPONSE LEN: %d", ret);
        ESP_LOGI(TAG, "RESPONSE: ");
        for(uint8_t i = 0; i < ret; i++)
            printf("%x ", buffer[i]);
            
        isStartSuccess = memcmp(buffer, expected_response, sizeof(expected_response)) == 0;   
        if(!isStartSuccess) {
            ESP_LOGI(TAG, "EXPECTED RESPONSE NOT FOUND");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        } else {
            gpio_set_direction(2, GPIO_MODE_OUTPUT);
            gpio_set_level(2, 1);
            gpio_set_level(LIDAR_MOTOR_CONTROL_PIN, 1);
        }

    } while(tryCount < 5 && !isStartSuccess);

    if(!isStartSuccess) {
        ESP_LOGE(TAG, "CAN NOT START LIDAR");
        return;
    }
}

void lidar_print(Lidar_Data point, char* pointName) {
    if(point.distant == 0)
        ESP_LOGE(pointName, "\nangle = %f\ndistant = %f\n", point.angle, point.distant);
    else
        ESP_LOGI(pointName, "\nangle = %f\ndistant = %f\n", point.angle, point.distant);
}

static void lidar_reset_value(Lidar_Data* value) {
    value->angle = 0;
    value->distant = 0;
}

static Lidar_Data lidar_convert_raw_data(uint8_t* raw_data) {
    Lidar_Data ret;
    uint16_t temp_distant = *(uint16_t*)(&raw_data[3]);
    ret.distant = (float)temp_distant / 4;

    uint16_t temp_angle_0 = (uint16_t)(raw_data[1]) >> 1;
    uint16_t temp_angle_1 = (uint16_t)(raw_data[2]) << 7;
    uint16_t temp_angle = temp_angle_0 | temp_angle_1;
    ret.angle = (float)temp_angle / 64;

    return ret;
}

static uint8_t lidar_is_raw_data_valid(uint8_t* raw_data) {
    uint16_t temp_distant = *(uint16_t*)(&raw_data[3]) / 4;
    return temp_distant != 0 && temp_distant < MAX_DISTANT; 
}