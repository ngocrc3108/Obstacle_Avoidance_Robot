#ifndef __CAR_H__
#define __CAR_H__

#include "wheel.h"

#define CAR_SPEED 50.0 // 0 -> 100

typedef struct {
    Wheel left;
    Wheel right;
} Car;

void car_init();
void car_go_forward();
void car_go_backward();
void car_turn_left();
void car_turn_right();
void car_stop();
void car_turn_by_angle(float angle);
void car_turn_by_angle_and_forward(float angle);
void car_set_speed(float speed);
#endif