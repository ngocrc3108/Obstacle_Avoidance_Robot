#include <stdio.h>
#include "./inc/wheel.h"

void app_main(void) {
    Wheel leftWheel = {
        .mcpwm_unit = MCPWM_UNIT_0,
        .mcpwm_timer = MCPWM_TIMER_0,
        .mcpwm_io_signals_A = MCPWM0A,
        .mcpwm_io_signals_B = MCPWM0B,
        .gpio_pwmA_out = WHEEL_GPIO_A_LEFT,
        .gpio_pwmB_out = WHEEL_GPIO_B_LEFT,
    };
    wheel_create(&leftWheel);
    wheel_set_speed(&leftWheel, 50.0);
    while(1) {
        wheel_forward(leftWheel);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        wheel_backward(leftWheel);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        wheel_stop(leftWheel);
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}