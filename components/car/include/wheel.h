#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#define WHEEL_GPIO_A_LEFT 27
#define WHEEL_GPIO_B_LEFT  26
#define WHEEL_GPIO_A_RIGHT 4
#define WHEEL_GPIO_B_RIGHT 18

typedef struct {
    mcpwm_unit_t mcpwm_unit;
    mcpwm_timer_t mcpwm_timer;
    mcpwm_io_signals_t mcpwm_io_signals_A;
    mcpwm_io_signals_t mcpwm_io_signals_B;
    int gpio_pwmA_out;
    int gpio_pwmB_out;
    float speed;
} Wheel;

/**
 * @brief motor moves in forward direction
 */
void wheel_forward(Wheel wheel);

/**
 * @brief motor moves in backward direction
 */
void wheel_backward(Wheel wheel);

/**
 * @brief motor stop
 */
void wheel_stop(Wheel wheel);

/**
 * @brief set motor speed
 * @param speed duty cycle = duty %
 */
void wheel_set_speed(Wheel *wheel, float speed);

/**
 * @brief Configure MCPWM module for brushed dc motor
 */
void wheel_create(Wheel *wheel);