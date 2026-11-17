#include "tmb12a05.h"

void TMB12A05_Init(TMB12A05* t, GPIO_TypeDef* port, uint16_t pin, uint8_t activeHigh) {
    t->port = port;
    t->pin = pin;
    t->activeHigh = activeHigh;
    t->isOn = 1;
    t->stopTime = 0;
    TMB12A05_Off(t);
}
void TMB12A05_On(TMB12A05* t) {
    if(t->isOn == 1) return;
    HAL_GPIO_WritePin(t->port, t->pin, SET);
    t->isOn = 1;
}
void TMB12A05_Off(TMB12A05* t) {
    if(t->isOn == 0) return;
    
    HAL_GPIO_WritePin(t->port, t->pin, RESET);
    t->isOn = 0;
}

void TMB12A05_Beep_Start(TMB12A05* t, uint16_t duration_ms) {
    TMB12A05_On(t);
    t->stopTime = HAL_GetTick() + duration_ms;
}

void TMB12A05_Update(TMB12A05* t) {
    if(!t->isOn) return;

    if(HAL_GetTick() >= t->stopTime) {
        TMB12A05_Off(t);
    }
}