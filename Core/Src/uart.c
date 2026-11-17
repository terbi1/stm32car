#include "uart.h"

static TaskHandle_t uartRxTaskHandle = NULL;

void Ring_Buffer_Init(Ring_Buffer* ring_buffer, uint8_t* buffer, uint16_t buffer_size) {
    ring_buffer->buffer = buffer; 
    ring_buffer->head = 0;
    ring_buffer->tail = 0;
    ring_buffer->buffer_size = buffer_size;
    ring_buffer->filled_size = 0;
}

bool Ring_Buffer_Pop(Ring_Buffer* ring_buffer, uint8_t* data) {
    if(!ring_buffer->filled_size) return false;

    *data = ring_buffer->buffer[ring_buffer->head];
    ring_buffer->head = (ring_buffer->head+1) % ring_buffer->buffer_size;
    --ring_buffer->filled_size;

    return true;
}

void Ring_Buffer_Push(Ring_Buffer* ring_buffer, uint8_t data) {
    ring_buffer->buffer[ring_buffer->tail] = data;
    ring_buffer->tail = (ring_buffer->tail+1) % ring_buffer->buffer_size;
    
    if((uint32_t)(ring_buffer->filled_size + 1) > ring_buffer->buffer_size) {
        ring_buffer->head = (ring_buffer->head+1) % ring_buffer->buffer_size;
    } else {
        ++ring_buffer->filled_size;
    }
}
bool Ring_Buffer_IsEmpty(Ring_Buffer* ring_buffer) {
    return (ring_buffer->filled_size == 0); 
}
bool Ring_Buffer_IsFull(Ring_Buffer* ring_buffer) {
    return (ring_buffer->filled_size == ring_buffer->buffer_size);
}
uint16_t Ring_Buffer_Size(Ring_Buffer* ring_buffer) {
    return ring_buffer->buffer_size;
}
uint16_t Ring_Buffer_Filled_Size(Ring_Buffer* ring_buffer) {
    return ring_buffer->filled_size;
}

UART u;

uint8_t data;

void UART_Init(UART_HandleTypeDef* huart, uint8_t* buffer, uint16_t buffer_size) {
    u.uart = huart;

    Ring_Buffer_Init(&u.buffer, buffer, buffer_size);

    HAL_UART_Receive_IT(u.uart, &data, 1);
}

void UART_SetRxTaskHandle(TaskHandle_t handle) {
    uartRxTaskHandle = handle;
}

void UART_Send_String(char* str) {
    // char* str;
    HAL_UART_Transmit(u.uart, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}
void UART_Send_Data(uint8_t* data, uint16_t len) {
    // HAL_UART_Transmit_IT(u.uart, data, len);
    HAL_UART_Transmit(u.uart, data, len, HAL_MAX_DELAY); // <--- CHANGE TO BLOCKING

}
void UART_Printf(const char* format);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if(huart->Instance == u.uart->Instance) {
        Ring_Buffer_Push(&u.buffer, data);

        if(uartRxTaskHandle != NULL) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            vTaskNotifyGiveFromISR(uartRxTaskHandle, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
        HAL_UART_Receive_IT(huart, &data, 1);
    }
}

bool UART_Read_Byte(uint8_t* data) {
    return Ring_Buffer_Pop(&u.buffer, data);
}
void UART_Read_Line(char* buffer, uint16_t max_len) {
    uint16_t size = u.buffer.filled_size;
    // for(int i - 0;)
}
uint16_t UART_Available() {
    return u.buffer.filled_size;
};

void UART_Flush_RX(UART* u);
void __io_putchar();