#ifndef __LED_H__
#define __LED_H__

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define LED_PIN             2

typedef enum {
    LED_OFF = 0,
    LED_ON,
    LED_BLINK, 
} LedState_t;

void led_init();
void led_set_state(LedState_t state);

#endif