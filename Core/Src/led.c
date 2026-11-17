#include "led.h"

void LED_Init(LED* led, GPIO_TypeDef* port, uint16_t pin, uint8_t activeHigh) {
    led->port = port;
    led->pin = pin;
    led->activeHigh = activeHigh;
    led->isOn = 1;
    led->stopTime = 0;
    led->interval = 0;
    led->blinkCycles = 0;
    led->repeat = 0;
    LED_Off(led);
}
void LED_On(LED* led) {
    if(led->isOn == 1) return;
    HAL_GPIO_WritePin(led->port, led->pin, SET);
    led->isOn = 1;
}
void LED_Off(LED* led) {
    if(led->isOn == 0) return;
    
    HAL_GPIO_WritePin(led->port, led->pin, RESET);
    led->isOn = 0;
}

void LED_Beep_Start(LED* led, uint16_t duration_ms) {
    LED_On(led);
    led->stopTime = HAL_GetTick() + duration_ms;
}

void LED_Toggle(LED* led) {
    HAL_GPIO_TogglePin(led->port, led->pin);
}


void LED_Update(LED* led) {
    if(!led->isOn) {
        if(!led->blinkCycles) return;

        if(HAL_GetTick() > led->stopTime + led->interval) {
            LED_Beep_Start(led, BLINK_TIME);
            led->blinkCycles -= led->repeat ? 0:1;
        }

        return;
    }

    if(HAL_GetTick() >= led->stopTime) {
        LED_Off(led);
    }
}

void LED_Cycle_Blink(LED *led, uint8_t cycles, uint32_t interval) {
    led->blinkCycles = cycles;
    led->interval = interval;
}

void LED_Repeat_Blink(LED *led, uint32_t interval) {
    led->repeat = 1;
    led->interval = interval;
    led->blinkCycles = 1;
}