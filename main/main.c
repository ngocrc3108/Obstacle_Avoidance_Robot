#include "car.h"
#include "lidar.h"

void app_main(void) {
    car_init();
    //car_turn_by_angle_and_forward(180);
    lidar_init();
    // car_set_speed(CAR_SPEED);
    // car_go_backward();
    // while(1) {
    //     car_turn_by_angle_and_forward(-90);
    //     vTaskDelay(4000 / portTICK_PERIOD_MS);
    //     car_stop();
    //     vTaskDelay(4000 / portTICK_PERIOD_MS);
    // }
}