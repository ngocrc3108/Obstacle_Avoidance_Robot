#include "uart.h"
#include "car.h"
#include "lidar.h"

void app_main(void) {    
    car_init();
    lidar_init();
}