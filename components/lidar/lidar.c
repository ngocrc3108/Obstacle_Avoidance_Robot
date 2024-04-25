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

static const char *TAG = "uart_events";

static void uart_event_task(void *pvParameters);
static void uart_init();
static void uart_enable_interrupt();

static QueueHandle_t lidar_uart_queue;
Lidar_Data lidar;

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t* uart_data = (uint8_t*) malloc(BUF_SIZE);
    int count  = 0;
    for (;;) {
        //Waiting for UART event.
        if (xQueueReceive(lidar_uart_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
            bzero(uart_data, BUF_SIZE);
            ESP_LOGI(TAG, "uart[%d] event:", LIDAR_UART_NUM);
            switch (event.type) {
            case UART_DATA:
                ESP_LOGI(TAG, "[UART DATA]: %d, COUNT: %d", event.size, count++);
                uart_read_bytes(LIDAR_UART_NUM, uart_data, event.size, portMAX_DELAY);

                // handle interrupt here
                for(int i = 0; i <= (int)event.size - LIDAR_DATA_PACKET_SIZE; i += LIDAR_DATA_PACKET_SIZE) {
                    uint16_t temp_distant = *(uint16_t*)(&uart_data[i + 3]);
                    lidar.distant = (float)temp_distant / 4;

                    uint16_t temp_angle_0 = (uint16_t)(uart_data[i+1]) >> 1;
                    uint16_t temp_angle_1 = (uint16_t)(uart_data[i+2]) << 7;
                    uint16_t temp_angle = temp_angle_0 | temp_angle_1;
                    lidar.angle = (float)temp_angle / 64;

                    ESP_LOGE("DEBUG", "i: %d", i);
                    lidar_print();
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
                ESP_LOGI(TAG, "ring buffer full");
                // If buffer full happened, you should consider increasing your buffer size
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(LIDAR_UART_NUM);
                xQueueReset(lidar_uart_queue);
                break;
            //Event of UART RX break detected
            case UART_BREAK:
                ESP_LOGI(TAG, "uart rx break");
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

    int ret = uart_write_bytes(LIDAR_UART_NUM, startCommand, sizeof(startCommand));
    ESP_LOGI(TAG, "WRITE CMD LEN: %d", ret);

    ret = uart_read_bytes(LIDAR_UART_NUM, buffer, 7, 5000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "READ RESPONSE LEN: %d", ret);
    ESP_LOGI(TAG, "RESPONSE: ");
    for(uint8_t i = 0; i < ret; i++)
        printf("%x ", buffer[i]);
        
    if(memcmp(buffer, expected_response, sizeof(expected_response)) != 0) {
        ESP_LOGI(TAG, "EXPECTED RESPONSE NOT FOUND");
        return;
    } else {
        gpio_set_level(LIDAR_MOTOR_CONTROL_PIN, 1);
    }

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

void lidar_print() {
    if(lidar.distant == 0)
        ESP_LOGE(TAG, "\nangle = %f\ndistant = %f\n", lidar.angle, lidar.distant);
    else
        ESP_LOGI(TAG, "\nangle = %f\ndistant = %f\n", lidar.angle, lidar.distant);
}