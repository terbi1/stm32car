#include "servo.h"

uint32_t Servo_Get_Timer(TIM_HandleTypeDef* htim) {
    uint32_t clk = 0;
    uint32_t presc = 0;
    if(htim->Instance == TIM1) {
        clk = HAL_RCC_GetPCLK2Freq();
        presc = RCC->CFGR & RCC_CFGR_PPRE2;

        if(presc != RCC_CFGR_PPRE2_DIV1) {
            clk *= 2;
        }
    } else {
        clk = HAL_RCC_GetPCLK1Freq();
        presc = RCC->CFGR & RCC_CFGR_PPRE1;

        if(presc != RCC_CFGR_PPRE1_DIV1) {
            clk *= 2;
        }
    }

    return clk;
}

void Servo_Init(Servo* s, TIM_HandleTypeDef* htim, uint32_t channel, uint16_t min_us, uint16_t max_us) {
    s->htim = htim;
    s->channel = channel;
    s->min_us = min_us;
    s->max_us = max_us;
    s->angle = 0;
    
    uint32_t clk = Servo_Get_Timer(htim);
    s->tick_hz = clk / (htim->Instance->PSC + 1);
    HAL_TIM_PWM_Start(htim, channel);
    // uint16_t step_per_degree = (max_us - min_us) / t_per_step / 180;
}
void Servo_Write_Angle(Servo* s, uint8_t angle) {
    if (angle < 0) angle = 0;
    if(angle > 180) angle = 180;
    // uint32_t t_per_step = (s->htim->Instance->ARR+1) / s->tick_hz * 10e6;
    uint32_t pulse_time = s->min_us + (s->max_us - s->min_us) * angle / 180;
    uint32_t pulse_step = (uint64_t)pulse_time * s->tick_hz / 1000000ULL;

    uint32_t arr = s->htim->Instance->ARR;
    if(pulse_step > arr) {
        pulse_step = arr;
    }

    __HAL_TIM_SET_COMPARE(s->htim, s->channel, pulse_step);
}

// 500us = 0
// 2500us = 180