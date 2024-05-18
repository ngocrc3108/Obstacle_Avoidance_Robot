#include "led.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

SemaphoreHandle_t led_semaphore;
LedState_t led_state = LED_OFF;

static void led_control_task(void* para);

void led_init() {
    gpio_set_direction(LED_PIN, GPIO_MODE_INPUT_OUTPUT);

    led_semaphore = xSemaphoreCreateMutex();

    xTaskCreate(led_control_task, "led_control_task", 1024, NULL, 5, NULL);
}

static void led_control_task(void* para) {
    for(;;) {
        xSemaphoreTake(led_semaphore, portMAX_DELAY);
        if(led_state == LED_OFF)
            gpio_set_level(LED_PIN, 0);
        else if(led_state == LED_ON)
            gpio_set_level(LED_PIN, 1);
        else if(led_state == LED_BLINK)
            gpio_set_level(LED_PIN, !gpio_get_level(LED_PIN));
        xSemaphoreGive(led_semaphore);

        vTaskDelay(pdMS_TO_TICKS(100));         
    }
}

void led_set_state(LedState_t state) {
    xSemaphoreTake(led_semaphore, portMAX_DELAY);
    led_state = state;
    xSemaphoreGive(led_semaphore);
}