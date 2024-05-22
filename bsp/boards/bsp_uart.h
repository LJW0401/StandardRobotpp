#ifndef BSP_UART_H
#define BSP_UART_H
#include "stm32f4xx_hal.h"
#include "struct_typedef.h"
#include "usart.h"

typedef UART_HandleTypeDef huart_t;
#define UART1 huart6 //(3pin)与板上的标识相对应
#define UART2 huart1 //(4pin)与板上的标识相对应

typedef enum __UartSendState{
    UART_SEND_FAIL = 0,
    UART_SEND_OK,
} UartSendState_e;

#endif  // BSP_UART_H
