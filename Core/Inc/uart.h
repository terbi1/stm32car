#ifndef UART_H
#define UART_H

#include "stm32f1xx_hal.h"
#include <stm32f1xx_hal_tim.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stm32f1xx_hal_uart.h>
#include "cmsis_os.h"


typedef struct {
    uint8_t* buffer;
    uint16_t head;
    uint16_t tail;
    uint16_t buffer_size;
    uint16_t filled_size;
} Ring_Buffer;

void Ring_Buffer_Init(Ring_Buffer* ring_buffer, uint8_t* buffer, uint16_t buffer_size);
bool Ring_Buffer_Pop(Ring_Buffer* ring_buffer, uint8_t* data);
void Ring_Buffer_Push(Ring_Buffer* ring_buffer, uint8_t data);
bool Ring_Buffer_IsEmpty(Ring_Buffer* ring_buffer);
bool Ring_Buffer_IsFull(Ring_Buffer* ring_buffer);
uint16_t Ring_Buffer_Size(Ring_Buffer* ring_buffer);
uint16_t Ring_Buffer_Filled_Size(Ring_Buffer* ring_buffer);

typedef struct {
    UART_HandleTypeDef* uart;

    Ring_Buffer buffer;
} UART;



extern UART u;

void UART_Init(UART_HandleTypeDef* huart, uint8_t* buffer, uint16_t buffer_size);
void UART_SetRxTaskHandle(TaskHandle_t handle);
void UART_Send_String(char* str);
void UART_Send_Data(uint8_t* data, uint16_t len);
void UART_Printf(const char* format);

void HAL_UART_RxCpltCallback();
bool UART_Read_Byte(uint8_t* data);
void UART_Read_Line(char* buffer, uint16_t max_len);
uint16_t UART_Available();

void UART_Flush_RX(UART* u);
void __io_putchar();

#endif