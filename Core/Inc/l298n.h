#ifndef L298N_H
#define L298N_H
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include <stm32f1xx_hal_tim.h>
#include <stdbool.h>

#define PWM_PERIOD 1000
#define GUARD_TIME_MS 80
#define BASE_PWM 50.0f
#define MAX_RPM 230.0f
#define MIN_RPM 115.0f
#define COUNTS_PER_REV 40
// #define Kp 0.0f
// #define Ki 0.0f
// #define Kff 1.0f

enum DIRECTION {
    FORWARD,
    BACKWARD,
    STOP
};

typedef struct{
    GPIO_TypeDef *forward_port;
    uint16_t forward_pin;

    GPIO_TypeDef *backward_port;
    uint16_t backward_pin;

    TIM_HandleTypeDef *htim;
    uint32_t channel;

    enum DIRECTION dir;
    
    uint32_t time_left;

} L298N_Motor;

void L298N_Motor_Init(L298N_Motor *m);
void L298N_Motor_Write(L298N_Motor *m, uint16_t pwm);
void L298N_Forward(L298N_Motor *m, uint8_t duty_pct);
void L298N_Backward(L298N_Motor *m, uint8_t duty_pct);
void L298N_Brake(L298N_Motor *m);
void L298N_Coast(L298N_Motor *m);
bool L298N_Motor_Check_Direction(L298N_Motor *m, enum DIRECTION);

// void L298N_Init(L298N* mL298N, GPIO_TypeDef *PORT, uint8_t* IN, uint8_t* OUT, uint8_t* EN);
// void L298N_WritePin(L298N* mL298N, uint8_t Pin);
// void L298N_TogglePin(L298N* mL298N, uint8_t Pin);
// int L298N_ReadPin(L298N* mL298N, uint8_t Pin);

#endif