#ifndef PI_CTL_H
#define PI_CTL_H
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include <stm32f1xx_hal_tim.h>
#include <stdbool.h>

#define Kff 1.0f
#define Kp 1.5f
#define Ki 0.4f

// extern float integral = 0;
typedef struct{
    // float Kff;
    // float Kp;
    // float Kd;
    float integral;
} PI_Controller;

typedef struct {
    int32_t tick;
    uint32_t t;
    float rpm;
} Speed;

// void PI_Controller_Init(float Kff, float Kp, float Kd);

float Feed_Forward(float base, float setpoint);
float Proportional(float error);
float Integral(float error, float dt, float current_integral, float output);
float System_Input(PI_Controller *p, float base, float setpoint, float error, float dt);
float Speed_Update(Speed* spd, int32_t tick, uint32_t t, uint32_t now);

#endif