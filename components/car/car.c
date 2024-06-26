#include "car.h"
#include <math.h>
#include "esp_log.h"

static Car car = {
    .left = {
        .mcpwm_unit = MCPWM_UNIT_0,
        .mcpwm_timer = MCPWM_TIMER_0,
        .mcpwm_io_signals_A = MCPWM0A,
        .mcpwm_io_signals_B = MCPWM0B,
        .gpio_pwmA_out = WHEEL_GPIO_A_LEFT,
        .gpio_pwmB_out = WHEEL_GPIO_B_LEFT,
    },
    .right = {
        .mcpwm_unit = MCPWM_UNIT_1,
        .mcpwm_timer = MCPWM_TIMER_1,
        .mcpwm_io_signals_A = MCPWM1A,
        .mcpwm_io_signals_B = MCPWM1B,
        .gpio_pwmA_out = WHEEL_GPIO_A_RIGHT,
        .gpio_pwmB_out = WHEEL_GPIO_B_RIGHT,
    }
}; 

void car_init() {
    wheel_create(&car.left);
    wheel_set_speed(&car.left, CAR_SPEED);

    wheel_create(&car.right);
    wheel_set_speed(&car.right, CAR_SPEED);
}

void car_go_forward() {
    car_set_speed(CAR_SPEED);

    wheel_forward(car.left);
    wheel_forward(car.right);
}

void car_go_backward() {
    car_set_speed(CAR_SPEED);

    wheel_backward(car.left);
    wheel_backward(car.right);    
}

void car_turn_left() {
    car_set_speed(CAR_SPEED);

    wheel_forward(car.right);
    wheel_stop(car.left); 
}

void car_turn_right() {
    car_set_speed(CAR_SPEED);
    wheel_forward(car.left);
    wheel_stop(car.right);
  
}

void car_stop() {
    wheel_stop(car.left);
    wheel_stop(car.right);
}

void car_turn_by_angle(float angle) {
    if(angle > 0) {
        car_turn_right();
        //vTaskDelay(angle / 80.0 * 1000 / portTICK_PERIOD_MS); // 1s = 80*
    } else if(angle < 0) {
        car_turn_left();
        //vTaskDelay(- angle / 80.0 * 1000 / portTICK_PERIOD_MS); // 1s = 80*        
    }
}

void car_turn_by_angle_and_forward(float angle) {
    if(angle > 0) {
        //ESP_LOGI("DEBUG", "turn right");
        wheel_set_speed(&car.left, CAR_SPEED + (100 - CAR_SPEED)*fabs(angle) / 180.0);
        wheel_set_speed(&car.right, CAR_SPEED);
    } else if(angle < 0) {
        //ESP_LOGI("DEBUG", "turn left");
        wheel_set_speed(&car.left, CAR_SPEED);
        wheel_set_speed(&car.right, CAR_SPEED + (100 - CAR_SPEED)*fabs(angle) / 180.0);        
    } else {
        //ESP_LOGI("DEBUG", "go forward");
        car_set_speed(CAR_SPEED);
    }

    wheel_forward(car.left);
    wheel_forward(car.right);
}

void car_set_speed(float speed) {
    wheel_set_speed(&car.left, speed);
    wheel_set_speed(&car.right, speed);
}