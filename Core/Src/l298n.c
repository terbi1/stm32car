#include <l298n.h>

void L298N_Motor_Init(L298N_Motor *m) {
    HAL_GPIO_WritePin(m->forward_port, m->forward_pin, RESET);
    HAL_GPIO_WritePin(m->backward_port, m->backward_pin, RESET);
    __HAL_TIM_SetCompare(m->htim, m->channel, 0);
    HAL_TIM_PWM_Start(m->htim, m->channel);
    m->dir = STOP;
    m->time_left = 0;
}

void L298N_Motor_Write(L298N_Motor *m, uint16_t pwm) {
     __HAL_TIM_SET_COMPARE(m->htim, m->channel, pwm);
}

void L298N_Forward(L298N_Motor *m, uint8_t duty_pct) {
    // L298N_Coast(m);

    while(!L298N_Motor_Check_Direction(m, FORWARD)) {

    }

    HAL_GPIO_WritePin(m->forward_port, m->forward_pin, RESET);
    HAL_GPIO_WritePin(m->backward_port, m->backward_pin, SET);
    __HAL_TIM_SetCompare(m->htim, m->channel, 0);    
    L298N_Motor_Write(m, duty_pct * PWM_PERIOD / 100);
    m->dir = FORWARD;
}

void L298N_Backward(L298N_Motor *m, uint8_t duty_pct) {
    // L298N_Coast(m);

    while(!L298N_Motor_Check_Direction(m, BACKWARD)) {
        
    }

    HAL_GPIO_WritePin(m->forward_port, m->forward_pin, SET);
    HAL_GPIO_WritePin(m->backward_port, m->backward_pin, RESET);
    __HAL_TIM_SetCompare(m->htim, m->channel, 0);    
    L298N_Motor_Write(m, duty_pct * PWM_PERIOD / 100);
    m->dir = BACKWARD;
}
void L298N_Brake(L298N_Motor *m) {
    HAL_GPIO_WritePin(m->forward_port, m->forward_pin, SET);
    HAL_GPIO_WritePin(m->backward_port, m->backward_pin, SET);
    __HAL_TIM_SetCompare(m->htim, m->channel, PWM_PERIOD);
    m->dir = STOP;
    // HAL_Delay(500);
}
void L298N_Coast(L298N_Motor *m) {
    HAL_GPIO_WritePin(m->forward_port, m->forward_pin, RESET);
    HAL_GPIO_WritePin(m->backward_port, m->backward_pin, RESET);
    __HAL_TIM_SetCompare(m->htim, m->channel, 0);
    m->dir = STOP;
    // HAL_Delay(500);
}

bool L298N_Motor_Check_Direction(L298N_Motor *m, enum DIRECTION new_dir) {
    if(m->dir == new_dir) {
        return true;
    }

    if(m->dir == STOP) {
        if(m->time_left == 0) return true;
        if((int32_t)(HAL_GetTick() - m->time_left) >= 0) return true;
        return false;
    }

    L298N_Coast(m);
    m->dir = STOP;
    m->time_left = HAL_GetTick() + GUARD_TIME_MS;

    return false;
}
