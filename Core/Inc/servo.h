#ifndef SERVO_H
#define SERVO_H
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include <stm32f1xx_hal_tim.h>
#include <stdbool.h>

typedef struct{
    TIM_HandleTypeDef *htim;
    uint32_t channel;

    uint8_t angle;

    uint16_t min_us;
    uint16_t max_us;
    uint32_t tick_hz;
} Servo;

void Servo_Init(Servo* s, TIM_HandleTypeDef* htim, uint32_t channel, uint16_t min_us, uint16_t max_us);
void Servo_Write_Angle(Servo* s, uint8_t angle);

#endif