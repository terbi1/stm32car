#ifndef TMB12A05_H
#define TMB12A05_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
    uint8_t isOn;
    uint8_t activeHigh;
    uint32_t stopTime;
} TMB12A05;

void TMB12A05_Init(TMB12A05* t, GPIO_TypeDef* port, uint16_t pin, uint8_t activeHigh);
void TMB12A05_On(TMB12A05* t);
void TMB12A05_Off(TMB12A05* t);
void TMB12A05_Toggle(TMB12A05* t);
void TMB12A05_Beep_Start(TMB12A05* t, uint16_t duration_ms);
void TMB12A05_Update(TMB12A05*t);

#endif