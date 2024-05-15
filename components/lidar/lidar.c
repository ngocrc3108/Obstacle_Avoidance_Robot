#include "lidar.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "hal/uart_hal.h"
#include "driver/gpio.h"
#include "math.h"
#include "car.h"

#define MAX_DISTANT 2000

static const char *TAG = "uart_events";

static void uart_event_task(void *pvParameters);
static void uart_init();
static void uart_enable_interrupt();
static void reset_lidar_value(Lidar_Data* value);
static Lidar_Data convert_raw_data_to_lidar_data(uint8_t* raw_data);
static uint8_t is_raw_data_valid(uint8_t* raw_data);

static QueueHandle_t lidar_uart_queue;
Lidar_Data firstPointInNewScan;
uint32_t point_count;
Lidar_Data currentPoint;
Lidar_Data lastPoint;
Lidar_Data point_A;
Lidar_Data point_B;

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t* uart_data = (uint8_t*) malloc(BUF_SIZE);
    int flag = 0; 
    for (;;) {
        //Waiting for UART event.
        if (xQueueReceive(lidar_uart_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
            bzero(uart_data, BUF_SIZE);
            switch (event.type) {
            case UART_DATA:
                uart_read_bytes(LIDAR_UART_NUM, uart_data, event.size, portMAX_DELAY);

                // handle interrupt here
                for(int i = 0; i <= (int)event.size - LIDAR_DATA_PACKET_SIZE; i += LIDAR_DATA_PACKET_SIZE) {
                    uint8_t* raw_data = &uart_data[i];

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
                        ESP_LOGI("DEBUG", "turn angle: %f", turnAngle);

                        // lidar_print(point_A, "Point A");
                        // lidar_print(point_B, "Point B");
                        // ESP_LOGI("DEBUG", "point_count: %ld\n", point_count);

                        point_count = 0;
                        reset_lidar_value(&firstPointInNewScan);

                        if (flag) {
                            if(fabs(turnAngle) < 10) {
                                // be hon 10 do thi vua di chuyen vua xoay
                                //car_turn_by_angle_and_forward(turnAngle);
                            }
                            else {
                                // lon hon 10 do thi dung yen, xoay tai cho
                                //car_turn_by_angle(turnAngle);
                            }
                            car_turn_by_angle(turnAngle);
                        }
                        else {
                            car_go_forward();
                        }
                        
                        reset_lidar_value(&point_A);
                        reset_lidar_value(&point_B);
                        flag = 0;
                    }
                    
                    if(!is_raw_data_valid(raw_data)) 
                        continue;

                    lastPoint = currentPoint;
                    currentPoint = convert_raw_data_to_lidar_data(raw_data);
                    //lidar_print(currentPoint, "currentPoint");
                    if ((currentPoint.angle < 20 || currentPoint.angle > 340) && currentPoint.distant < 500) {
                        flag = 1;
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

                break;
            //Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                ESP_LOGI(TAG, "hw fifo overflow");
                // If fifo overflow happened, you should consider adding flow control for your application.
                // The ISR has already reset the rx FIFO,
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(LIDAR_UART_NUM);
                xQueueReset(lidar_uart_queue);
                break;
            //Event of UART ring buffer full
            case UART_BUFFER_FULL:
                ESP_LOGE(TAG, "ring buffer full");
                // If buffer full happened, you should consider increasing your buffer size
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(LIDAR_UART_NUM);
                xQueueReset(lidar_uart_queue);
                break;
            //Event of UART RX break detected
            case UART_BREAK:
                ESP_LOGE(TAG, "uart rx break");
                break;
            //Event of UART parity check error
            case UART_PARITY_ERR:
                ESP_LOGI(TAG, "uart parity error");
                break;
            //Event of UART frame error
            case UART_FRAME_ERR:
                ESP_LOGI(TAG, "uart frame error");
                break;
            //Others
            default:
                ESP_LOGI(TAG, "uart event type: %d", event.type);
                break;
            }
        }
    }
    free(uart_data);
    uart_data = NULL;
    vTaskDelete(NULL);
}

void lidar_init(void) {
    esp_log_level_set(TAG, ESP_LOG_INFO);

    gpio_set_direction(LIDAR_MOTOR_CONTROL_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LIDAR_MOTOR_CONTROL_PIN, 0);

    uart_init();

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

    uart_flush_input(LIDAR_UART_NUM);
    xQueueReset(lidar_uart_queue);

    uart_enable_interrupt();
}

static void uart_init() {
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    //Install UART driver, and get the queue.
    uart_driver_install(LIDAR_UART_NUM, BUF_SIZE, 0, LIDAR_UART_QUEUE_SIZE, &lidar_uart_queue, 0);
    uart_param_config(LIDAR_UART_NUM, &uart_config);

    uart_set_pin(LIDAR_UART_NUM, LIDAR_UART_TX, LIDAR_UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static void uart_enable_interrupt() {
    uart_intr_config_t uart_intr = {
        .rx_timeout_thresh = 1,
        .rxfifo_full_thresh  = 1,
        .intr_enable_mask = UART_INTR_RXFIFO_TOUT
    };
    uart_intr_config(LIDAR_UART_NUM, &uart_intr);
    uart_enable_rx_intr(LIDAR_UART_NUM);

    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
}

void lidar_print(Lidar_Data point, char* pointName) {
    if(point.distant == 0)
        ESP_LOGE(pointName, "\nangle = %f\ndistant = %f\n", point.angle, point.distant);
    else
        ESP_LOGI(pointName, "\nangle = %f\ndistant = %f\n", point.angle, point.distant);
}

static void reset_lidar_value(Lidar_Data* value) {
    value->angle = 0;
    value->distant = 0;
}

static Lidar_Data convert_raw_data_to_lidar_data(uint8_t* raw_data) {
    Lidar_Data ret;
    uint16_t temp_distant = *(uint16_t*)(&raw_data[3]);
    ret.distant = (float)temp_distant / 4;

    uint16_t temp_angle_0 = (uint16_t)(raw_data[1]) >> 1;
    uint16_t temp_angle_1 = (uint16_t)(raw_data[2]) << 7;
    uint16_t temp_angle = temp_angle_0 | temp_angle_1;
    ret.angle = (float)temp_angle / 64;

    return ret;
}

static uint8_t is_raw_data_valid(uint8_t* raw_data) {
    uint16_t temp_distant = *(uint16_t*)(&raw_data[3]) / 4;
    return temp_distant != 0 && temp_distant < MAX_DISTANT; 
}