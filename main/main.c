#include <stdio.h>
#include "car.h"
#include "lidar.h"
#include "driver/uart.h"

void app_main(void) {
    //car_init();
    lidar_init();
    while(1) {
        // car_go_forward();
        // vTaskDelay(2000 / portTICK_PERIOD_MS);
        // car_go_backward();
        // vTaskDelay(2000 / portTICK_PERIOD_MS);
        // car_turn_left();
        // vTaskDelay(1000 / portTICK_PERIOD_MS);
        // car_turn_right();
        // vTaskDelay(1000 / portTICK_PERIOD_MS);
        // car_stop();
        // vTaskDelay(4000 / portTICK_PERIOD_MS);
        // car_turn_by_angle(40);
        // car_go_forward();
        // vTaskDelay(2000 / portTICK_PERIOD_MS);
        // car_turn_by_angle(-90);
        // car_go_forward();
        // vTaskDelay(2000 / portTICK_PERIOD_MS);
        // car_stop();
        vTaskDelay(4000 / portTICK_PERIOD_MS);

    }
}