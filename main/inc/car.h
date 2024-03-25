#ifndef __CAR_H__
#define __CAR_H__

#define CAR_GPIO_LEFT
#define CAR_GPIO_RIGH

typedef struct {
    unsigned char gpio_left;
    unsigned char gpio_right;
} Car;

static Car car;

void car_init();
void car_go_forward();
void car_go_backward();
void car_turn_left();
void car_turn_right();

#endif