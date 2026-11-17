#ifndef LED_H
#define LED_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

#define BLINK_TIME 100

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
    uint8_t isOn;
    uint8_t activeHigh;
    uint32_t stopTime;
    uint32_t interval;
    uint8_t blinkCycles;
    uint8_t repeat;

} LED;

void LED_Init(LED* led, GPIO_TypeDef* port, uint16_t pin, uint8_t activeHigh);
void LED_On(LED* led);
void LED_Off(LED* led);
void LED_Toggle(LED* led);
void LED_Beep_Start(LED* led, uint16_t duration_ms);
void LED_Update(LED*led);
void LED_Cycle_Blink(LED *led, uint8_t cycles, uint32_t interval);
void LED_Repeat_Blink(LED *led, uint32_t interval);

#endif