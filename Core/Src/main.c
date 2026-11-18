/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "l298n.h"
#include "servo.h"
#include "tmb12a05.h"
#include "uart.h"
#include "pi_controller.h"
#include "autonomous.h"
#include "led.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osMessageQId uartQueueHandle;
/* USER CODE BEGIN PV */
osThreadId motorControlTaskHandle;
osThreadId ultrasonicTaskHandle;
osThreadId commandTaskHandle;
osThreadId telemetryTaskHandle;

QueueHandle_t telemQueue;

typedef struct
{
  float rpm_L;
  float rpm_R;
  float pwm_L;
  float pwm_R;
} TelemetryData;

volatile uint8_t rxData;
volatile uint8_t pendingCmd = 0;
volatile int32_t left_ticks = 0;
volatile int32_t right_ticks = 0;
volatile uint8_t turning_ticks = 0;

volatile float target_rpm = 200.0f;
uint8_t isIncreasing = 1;
uint8_t currentAngle = 0;
uint8_t buffer[256];
uint8_t isAutonomous = 0;
uint8_t isBlocked = 0;
uint8_t sonarScan = 0;

// enum DIR
// {
//   IDLE,
//   FRONT,
//   LEFT,
//   RIGHT,
//   BACK
// };

// enum CAR_STATE {
//   SCAN,
//   MOVE
// };

volatile uint8_t looKingDirection = FRONT;
// volatile uint8_t servoDirection = FRONT;
volatile uint8_t goingDirection = FRONT;
volatile uint8_t carState = MOVE;
volatile uint8_t scanDone = 0;

TMB12A05 t = {0};
LED led_left;
LED led_right;
Servo s;
L298N_Motor left_motor;
L298N_Motor right_motor;
PI_Controller ctl_L;
PI_Controller ctl_R;
AutonomousController auto_ctl;

volatile int8_t motor_x = 0;
volatile int8_t motor_y = 0;

volatile uint32_t L_t = 0, L_dt = 0, R_t = 0, R_dt = 0;
// uint8_t data;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);

/* USER CODE BEGIN PFP */
void StartMotorControlTask(void const *argument);
void StartUltrasonicTask(void const *argument);
void StartCommandTask(void const *argument);
void StartTelemetryTask(void const *argument);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint32_t now = DWT->CYCCNT;
  if (GPIO_Pin == GPIO_PIN_12)
  {
    // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    L_dt = now - L_t;
    L_t = now;
    ++left_ticks;
    // turning_ticks += (goingDirection == RIGHT || goingDirection == BACK) ? 1:0;
    auto_ctl.turning_ticks += (auto_ctl.goingDirection == RIGHT || auto_ctl.goingDirection == BACK) ? 1:0;
  }
  else if (GPIO_Pin == GPIO_PIN_11)
  {
    R_dt = now - R_t;
    R_t = now;
    ++right_ticks;
    // turning_ticks += (goingDirection == LEFT) ? 1:0;
    auto_ctl.turning_ticks += (auto_ctl.goingDirection == LEFT || auto_ctl.goingDirection == BACK) ? 1:0;
  }

  else
  {
    __NOP();
  }
}

void delay_us(uint16_t time_us)
{
  __HAL_TIM_SET_COUNTER(&htim1, 0);

  while (htim1.Instance->CNT < time_us)
    ;
}

void delay_ms(uint16_t time_ms)
{
  for (uint16_t i = 0; i < time_ms; ++i)
  {
    delay_us(1000);
  }
}

// void delay (uint16_t time)
// {
// 	__HAL_TIM_SET_COUNTER(&htim1, 0);
// 	while (__HAL_TIM_GET_COUNTER (&htim1) < time);
// }

uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0; // is the first value captured ?
uint32_t Distance = 400;
uint32_t leftDistance = 0;
uint32_t rightDistance = 0;

#define TRIG_PIN GPIO_PIN_7
#define TRIG_PORT GPIOA

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1 && htim->Instance == TIM1) // if the interrupt source is channel1
  {
    if (Is_First_Captured == 0) // if the first value is not captured
    {
      IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
      Is_First_Captured = 1;                                    // set the first captured as true
      // Now change the polarity to falling edge
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
    }

    else if (Is_First_Captured == 1) // if the first is already captured
    {
      IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read second value
      __HAL_TIM_SET_COUNTER(htim, 0);                           // reset the counter

      if (IC_Val2 > IC_Val1)
      {
        Difference = IC_Val2 - IC_Val1;
      }

      else if (IC_Val1 > IC_Val2)
      {
        Difference = (0xffff - IC_Val1) + IC_Val2;
      }

      Distance = Difference * .034 / 2;
      Is_First_Captured = 0; // set it back to false

      if (isAutonomous)
      {
        if(auto_ctl.looKingDirection == FRONT) {
          auto_ctl.frontDistance = Distance;
        }
        else if (!auto_ctl.leftDistance && auto_ctl.looKingDirection == LEFT)
        {
          // leftDistance = Distance;
          auto_ctl.leftDistance = Distance;
        }
        else if (!auto_ctl.rightDistance && auto_ctl.looKingDirection == RIGHT)
        {
          // rightDistance = Distance;
          auto_ctl.rightDistance = Distance;
        }
      }

      // set polarity to rising edge
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
      __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
    }
  }
}

uint32_t pulse_us = 0;

void HCSR04_Read(void)
{
  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET); // pull the TRIG pin HIGH
  // delay_us(10);                                           // wait for 10 us
  for (volatile uint16_t i = 0; i < 720; ++i)
  {
    __NOP();
  }
  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET); // pull the TRIG pin low

  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  // TMB12A05 t = {0};
  // Servo s;
  // L298N_Motor left_motor;
  left_motor.htim = &htim2;
  left_motor.channel = TIM_CHANNEL_1;
  left_motor.forward_port = GPIOA;
  left_motor.forward_pin = GPIO_PIN_2;
  left_motor.backward_port = GPIOA;
  left_motor.backward_pin = GPIO_PIN_3;
  // L298N_Motor right_motor;
  right_motor.htim = &htim2;
  right_motor.channel = TIM_CHANNEL_2;
  right_motor.forward_port = GPIOA;
  right_motor.forward_pin = GPIO_PIN_4;
  right_motor.backward_port = GPIOA;
  right_motor.backward_pin = GPIO_PIN_5;
  // uint8_t data;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // 72 MHz

  Servo_Init(&s, &htim3, TIM_CHANNEL_1, 500, 2500);
  // uint8_t buffer[256];
  UART_Init(&huart1, buffer, 256);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  L298N_Motor_Init(&left_motor);
  L298N_Motor_Init(&right_motor);
  TMB12A05_Init(&t, GPIOB, GPIO_PIN_0, 1);
  LED_Init(&led_left, GPIOB, GPIO_PIN_10, 1);
  LED_Init(&led_right, GPIOB, GPIO_PIN_1, 1);
  AutonomousController_Init(&auto_ctl);
  telemQueue = xQueueCreate(1, sizeof(TelemetryData));
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of uartQueue */
  osMessageQDef(uartQueue, 16, uint16_t);
  uartQueueHandle = osMessageCreate(osMessageQ(uartQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, StartTask02, osPriorityIdle, 0, 128);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadDef(motorControlTask, StartMotorControlTask, osPriorityRealtime, 0, 128);
  motorControlTaskHandle = osThreadCreate(osThread(motorControlTask), NULL);

  osThreadDef(ultrasonicControlTask, StartUltrasonicTask, osPriorityHigh, 0, 128);
  ultrasonicTaskHandle = osThreadCreate(osThread(ultrasonicControlTask), NULL);

  osThreadDef(commandTask, StartCommandTask, osPriorityAboveNormal, 0, 512);
  commandTaskHandle = osThreadCreate(osThread(commandTask), NULL);
  UART_SetRxTaskHandle((TaskHandle_t)commandTaskHandle);

  osThreadDef(telemetryTask, StartTelemetryTask, osPriorityNormal, 0, 256);
  telemetryTaskHandle = osThreadCreate(osThread(telemetryTask), NULL);
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA2 PA3 PA4 PA5
                           PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void StartMotorControlTask(void const *argument)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(100);
  int32_t last_L = 0;
  int32_t last_R = 0;

  Speed spd_L = {0};
  Speed spd_R = {0};

  for (;;)
  {
    if (isAutonomous)
    {
      AutonomousController_Update(&auto_ctl); 
    }

    if (pendingCmd == 'M' || isAutonomous)
    {
      if(auto_ctl.carState == SCAN) {
        L298N_Coast(&right_motor);
        L298N_Coast(&left_motor);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        continue;
      }

      if (Distance <= 15 && (motor_x - motor_y) > 0 && (-motor_x - motor_y) > 0 && !sonarScan)
      {
        L298N_Brake(&right_motor);
        L298N_Brake(&left_motor);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        continue;
      }

      float target_rpm_L = 0;
      float target_rpm_R = 0;

      if (!isAutonomous)
      {
        target_rpm_L = (motor_x - motor_y) / 100.0f * MAX_RPM;
        target_rpm_R = (-motor_x - motor_y) / 100.0f * MAX_RPM;
      }
      else
      {
        target_rpm_L = AUTO_PWM / 100.0f * MAX_RPM;
        target_rpm_R = AUTO_PWM / 100.0f * MAX_RPM;
      }

      taskENTER_CRITICAL();
      uint32_t tL = L_t;
      uint32_t tR = R_t;
      uint32_t now = DWT->CYCCNT;
      taskEXIT_CRITICAL();

      float rpm_L = Speed_Update(&spd_L, left_ticks, tL, now);
      float rpm_R = Speed_Update(&spd_R, right_ticks, tR, now);

      float pwm_L = System_Input(&ctl_L, BASE_PWM, (fabsf(target_rpm_L) - MIN_RPM) / (MAX_RPM - MIN_RPM) * (100.0f - BASE_PWM), (fabsf(target_rpm_L) - rpm_L) / (MAX_RPM - MIN_RPM) * (100.0f - BASE_PWM), 0.1f);

      float pwm_R = System_Input(&ctl_R, BASE_PWM, (fabsf(target_rpm_R) - MIN_RPM) / (MAX_RPM - MIN_RPM) * (100.0f - BASE_PWM), (fabsf(target_rpm_R) - rpm_R) / (MAX_RPM - MIN_RPM) * (100.0f - BASE_PWM), 0.1f);

      if (!isAutonomous)
      {
        if (target_rpm_L > 0)
        {
          L298N_Forward(&left_motor, (uint8_t)pwm_L);
        }
        else if (target_rpm_L < 0)
        {
          L298N_Backward(&left_motor, (uint8_t)pwm_L);
        }
        else
        {
          L298N_Coast(&left_motor);
        }

        if (target_rpm_R > 0)
        {
          L298N_Forward(&right_motor, (uint8_t)pwm_R);
        }
        else if (target_rpm_R < 0)
        {
          L298N_Backward(&right_motor, (uint8_t)pwm_R);
        }
        else
        {
          L298N_Coast(&right_motor);
        }
      } else if (auto_ctl.carState == MOVE){
        switch (auto_ctl.goingDirection)
        {
        case FRONT:
          if(auto_ctl.isBlocked) break;
          L298N_Forward(&left_motor, (uint8_t)pwm_L);
          L298N_Forward(&right_motor, (uint8_t)pwm_R);
          break;
        case RIGHT:
          L298N_Forward(&left_motor, (uint8_t)pwm_L);
          L298N_Backward(&right_motor, (uint8_t)pwm_R);
          // ctl_R.integral = 0;
          break;
        case LEFT:
          L298N_Forward(&right_motor, (uint8_t)pwm_R);
          L298N_Backward(&left_motor, (uint8_t)pwm_L);
          // ctl_L.integral = 0;
          break;
        case BACK:
          L298N_Backward(&left_motor, (uint8_t)pwm_L);
          L298N_Backward(&right_motor, (uint8_t)pwm_R);
          break;
        case IDLE:
          L298N_Brake(&left_motor);
          L298N_Brake(&right_motor);
          ctl_L.integral = 0;
          ctl_R.integral = 0;
          break;
        default:
          break;
        }
      }

      TelemetryData currentData;
      currentData.pwm_L = pwm_L;
      currentData.pwm_R = pwm_R;
      currentData.rpm_L = rpm_L;
      currentData.rpm_R = rpm_R;

      xQueueOverwrite(telemQueue, &currentData);
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void StartUltrasonicTask(void const *argument)
{
  for (;;)
  {
    HCSR04_Read();
    osDelay(100);
  }
}

void StartCommandTask(void const *argument)
{
  uint8_t byte;
  uint8_t in_M = 0;
  uint8_t len = 0;
  char line[32];

  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    while (UART_Read_Byte(&byte))
    {
      if (byte == 'M')
      {
        in_M = 1;
        len = 0;
        continue;
      }

      if (byte == 'H' 
        || byte == 'G' 
        || byte == 'F'
        || byte == 'J'
        || byte == 'K'
        || byte == 'I'
        || byte == 'Q')
      {
        pendingCmd = byte;
        continue;
      }

      if (!in_M)
        continue;

      if (byte == '\r' || byte == '\n')
      {
        if (len > 0)
        {
          line[len] = '\0';
          int x, y;
          if (sscanf(line, "%d %d", &x, &y) == 2)
          {
            taskENTER_CRITICAL();
            motor_x = x;
            motor_y = y;
            pendingCmd = 'M';
            if (x == 0 && y == 0)
            {
              ctl_L.integral = 0;
              ctl_R.integral = 0;
            }
            taskEXIT_CRITICAL();
            // osDelay(200);
          }
        }
        in_M = 0;
        len = 0;
      }
      else if (len < sizeof(line) - 1)
        line[len++] = byte;
      else
        in_M = 0;
    }
  }
}

void StartTelemetryTask(void const *argument)
{
  TelemetryData telemData;
  char temp[256];
  for (;;)
  {
    if (xQueueReceive(telemQueue, &telemData, pdMS_TO_TICKS(500)) == pdPASS)
    {

      // int c = snprintf(temp, sizeof(temp), "rpm_R:%d rpm_L:%d pwm_R:%d pwm_L:%d ticks_L:%d ticks_R:%d angle:%d distance:%d led_r:%d led_l:%d sonar: %d autonomous:%d\r\n", (int)(telemData.rpm_R * 10), (int)(telemData.rpm_L * 10), (int)(telemData.pwm_R * 10), (int)(telemData.pwm_L * 10), left_ticks, right_ticks, (int)currentAngle, (unsigned)Distance, led_right.isOn, led_left.isOn, sonarScan, isAutonomous);
      int c = snprintf(temp, sizeof(temp), "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n", (int)(telemData.rpm_R * 10), (int)(telemData.rpm_L * 10), (int)(telemData.pwm_R * 10), (int)(telemData.pwm_L * 10), left_ticks, right_ticks, (int)currentAngle, (int)Distance, led_right.isOn, led_left.isOn, sonarScan, isAutonomous);
      // int c = snprintf(temp, sizeof(temp), "left:%d right:%d\r\n", left_ticks, right_ticks);
      //   int c = snprintf(temp, sizeof(temp), "left:%d right:%d\r\n", left_ticks, right_ticks);
      UART_Send_Data(temp, c);
    }
    // int c = snprintf(temp, sizeof(temp), "left:%d right:%d\r\n", left_ticks, right_ticks);
    // int c = snprintf(temp, sizeof(temp), "%ld %ld\r\n", (int)(currentAngle), (unsigned)Distance);
    // int c = snprintf(temp, sizeof(temp), "%d %d\r\n", motor_x, motor_y);
    // UART_Send_Data(temp, c);
    // osDelay(500);
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  uint8_t horn_flag = 1;
  for (;;)
  {
    TMB12A05_Update(&t);
    LED_Update(&led_left);
    LED_Update(&led_right);
    
    if (isAutonomous)
    {
      if(auto_ctl.isBlocked && horn_flag) {
        TMB12A05_Beep_Start(&t, 10);
        horn_flag = 0;
      }

      if (auto_ctl.isBlocked && !auto_ctl.leftDistance && !auto_ctl.rightDistance && auto_ctl.carState == SCAN)
      {
        Servo_Write_Angle(&s, 135);
        osDelay(500);
        auto_ctl.looKingDirection = LEFT;
        osDelay(200);

        Servo_Write_Angle(&s, 45);
        osDelay(500);
        auto_ctl.looKingDirection = RIGHT;
        osDelay(200);

        auto_ctl.looKingDirection = FRONT;
        Servo_Write_Angle(&s, 90);

        auto_ctl.scanDone = 1;
      }
      else if(auto_ctl.isBlocked && auto_ctl.carState == MOVE) {
        if(auto_ctl.goingDirection == LEFT) {
          LED_Cycle_Blink(&led_left, 10, 100);
        } else if (auto_ctl.goingDirection == RIGHT) {
          LED_Cycle_Blink(&led_right, 10, 100);
        }
      }
      else if(!auto_ctl.isBlocked)
      {
        Servo_Write_Angle(&s, 90);
        horn_flag = 1;
      }
    } else if(!sonarScan) {
      Servo_Write_Angle(&s, 90);
    }

    switch (pendingCmd)
    {
    case 'H':
      TMB12A05_Beep_Start(&t, 10);
      break;
    case 'G':
      LED_Toggle(&led_left);
      break;
    case 'F':
      LED_Toggle(&led_right);
      break;
    case 'J':
      if(led_left.repeat) {
        LED_Stop_Blink(&led_left);
      } else LED_Repeat_Blink(&led_left, 100);
      break;
    case 'K':
      if(led_right.repeat) {
        LED_Stop_Blink(&led_right);
      } else LED_Repeat_Blink(&led_right, 100);
      break;
    case 'I':
      isAutonomous = 1 - isAutonomous;
      sonarScan = 0;
      break;
    case 'Q':
      if(isAutonomous) break;
      sonarScan = 1 - sonarScan;
      break;
    default:
      break;
    }
    if(pendingCmd != 'M') {
      pendingCmd = 0;
    }
    // if (pendingCmd == 'H' || pendingCmd == 'G' || pendingCmd == 'F')
    // {
    //   if (pendingCmd == 'H')
    //   {
    //     TMB12A05_Beep_Start(&t, 10);
    //   }
    //   else if (pendingCmd == 'G')
    //   {
    //     LED_Toggle(&led_left);
    //   } else if(pendingCmd == 'F') {
    //     LED_Toggle(&led_right);
    //   }

    //   pendingCmd = 0;
    // }
    if (sonarScan) {
      Servo_Write_Angle(&s, (uint8_t)currentAngle);
      if(isIncreasing) {
        ++currentAngle;
        if(currentAngle >= 180) {
          isIncreasing = 0;
        }
      } else {
        --currentAngle;
        if(currentAngle <= 0){
          isIncreasing = 1;
        }
      }
    }

    osDelay(50);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
 * @brief Function implementing the myTask02 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
