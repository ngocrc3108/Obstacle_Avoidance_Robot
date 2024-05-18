#include "uart.h"
#include "car.h"
#include "lidar.h"
#include "led.h"

void app_main(void) {    
    led_init();
    car_init();
    lidar_init();
}