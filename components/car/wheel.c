#include "wheel.h"

/**
 * @brief motor moves in forward direction
 */
void wheel_forward(Wheel wheel)
{
    mcpwm_set_signal_low(wheel.mcpwm_unit, wheel.mcpwm_timer, MCPWM_OPR_B);
    mcpwm_set_duty(wheel.mcpwm_unit, wheel.mcpwm_timer, MCPWM_OPR_A, wheel.speed);
    mcpwm_set_duty_type(wheel.mcpwm_unit, wheel.mcpwm_timer, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}

/**
 * @brief motor moves in backward direction
 */
void wheel_backward(Wheel wheel)
{
    mcpwm_set_signal_low(wheel.mcpwm_unit, wheel.mcpwm_timer, MCPWM_OPR_A);
    mcpwm_set_duty(wheel.mcpwm_unit, wheel.mcpwm_timer, MCPWM_OPR_B, wheel.speed);
    mcpwm_set_duty_type(wheel.mcpwm_unit, wheel.mcpwm_timer, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);  //call this each time, if operator was previously in low/high state
}

/**
 * @brief motor stop
 */
void wheel_stop(Wheel wheel)
{
    mcpwm_set_signal_low(wheel.mcpwm_unit, wheel.mcpwm_timer, MCPWM_OPR_A);
    mcpwm_set_signal_low(wheel.mcpwm_unit, wheel.mcpwm_timer, MCPWM_OPR_B);
}

/**
 * @brief set motor speed
 * @param speed duty cycle = duty %
 */
void wheel_set_speed(Wheel *wheel, float speed) {
    wheel->speed = speed;
}

/**
 * @brief Configure MCPWM module for brushed dc motor
 */
void wheel_create(Wheel *wheel) {

    //1. mcpwm gpio initialization
    printf("initializing mcpwm gpio...\n");
    mcpwm_gpio_init(wheel->mcpwm_unit, wheel->mcpwm_io_signals_A, wheel->gpio_pwmA_out);
    mcpwm_gpio_init(wheel->mcpwm_unit, wheel->mcpwm_io_signals_B, wheel->gpio_pwmB_out);

    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm...\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;    //frequency = 500Hz,
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(wheel->mcpwm_unit, wheel->mcpwm_timer, &pwm_config);    //Configure PWMxA & PWMxB with above settings
}