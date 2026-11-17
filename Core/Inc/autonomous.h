#ifndef AUTONOMOUS_H
#define AUTONOMOUS_H
#include <stdint.h>
#include "l298n.h"
#include "stm32f1xx_hal.h"
#include <stm32f1xx_hal_tim.h>
#include <stdbool.h>

// #define PWM_PERIOD 1000
// #define GUARD_TIME_MS 80
// #define BASE_PWM 50.0f
// #define MAX_RPM 230.0f
// #define MIN_RPM 115.0f
// #define COUNTS_PER_REV 40
#define AUTO_PWM 55.0f
#define AUTO_CRITICAL_DISTANCE 15
// #define Kp 0.0f
// #define Ki 0.0f
// #define Kff 1.0f

enum DIR
{
  IDLE,
  FRONT,
  LEFT,
  RIGHT,
  BACK
};

enum CAR_STATE {
  SCAN,
  MOVE
};

typedef struct{
    volatile uint8_t looKingDirection;
    // volatile uint8_t servoDirection;
    volatile uint8_t goingDirection;
    volatile uint8_t carState;
    volatile uint8_t scanDone;
    volatile uint8_t turning_ticks;
    uint32_t frontDistance;
    uint32_t leftDistance;
    uint32_t rightDistance;    
    uint8_t isBlocked;
} AutonomousController;

void AutonomousController_Init(AutonomousController *autoCtl);
void AutonomousController_NextState();
void AutonomousController_Update(AutonomousController *autoCtl);
// void L298N_Init(L298N* mL298N, GPIO_TypeDef *PORT, uint8_t* IN, uint8_t* OUT, uint8_t* EN);
// void L298N_WritePin(L298N* mL298N, uint8_t Pin);
// void L298N_TogglePin(L298N* mL298N, uint8_t Pin);
// int L298N_ReadPin(L298N* mL298N, uint8_t Pin);

#endif