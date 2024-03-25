#include <stdio.h>
#include "./inc/car.h"

void app_main(void) {
    car_init();
    while(1) {
        car_go_forward();
        vTaskDelay(4000 / portTICK_PERIOD_MS);
        car_go_backward();
        vTaskDelay(4000 / portTICK_PERIOD_MS);
        car_turn_left();
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        car_turn_right();
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        car_stop();
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}