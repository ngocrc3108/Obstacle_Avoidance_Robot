#include "uart.h"

#include "driver/uart.h"
#include "hal/uart_hal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "lidar.h"
#include "connection/socket.h"

static void uart_event_task(void *pvParameters);

QueueHandle_t uart_queue;
static const char *TAG = "uart_events";

void uart_init() {
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
    uart_driver_install(LIDAR_UART_NUM, BUF_SIZE, 0, LIDAR_UART_QUEUE_SIZE, &uart_queue, 0);
    uart_param_config(LIDAR_UART_NUM, &uart_config);

    uart_set_pin(LIDAR_UART_NUM, LIDAR_UART_TX, LIDAR_UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void uart_enable_interrupt() {
    uart_flush_input(LIDAR_UART_NUM);
    xQueueReset(uart_queue);

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

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t* uart_data = (uint8_t*) malloc(BUF_SIZE);
    for (;;) {
        //Waiting for UART event.
        if (xQueueReceive(uart_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
            bzero(uart_data, BUF_SIZE);
            switch (event.type) {
            case UART_DATA:
                uart_read_bytes(LIDAR_UART_NUM, uart_data, event.size, portMAX_DELAY);

                socket_send_async(uart_data, event.size);

                for(int i = 0; i <= (int)event.size - LIDAR_DATA_PACKET_SIZE; i += LIDAR_DATA_PACKET_SIZE)
                    lidar_handler(&uart_data[i]);
                break;
            //Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                ESP_LOGI(TAG, "hw fifo overflow");
                // If fifo overflow happened, you should consider adding flow control for your application.
                // The ISR has already reset the rx FIFO,
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(LIDAR_UART_NUM);
                xQueueReset(uart_queue);
                break;
            //Event of UART ring buffer full
            case UART_BUFFER_FULL:
                ESP_LOGE(TAG, "ring buffer full");
                // If buffer full happened, you should consider increasing your buffer size
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(LIDAR_UART_NUM);
                xQueueReset(uart_queue);
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