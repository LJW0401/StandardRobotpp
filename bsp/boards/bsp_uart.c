/**
 * @brief uart板级支持包
 * @author Penguin
 */
#include "bsp_uart.h"

UartSendState_e UartSendTxMessage(
    UART_HandleTypeDef * huart, uint8_t * pData, uint16_t Size, uint32_t Timeout)
{
    uint8_t cnt = 5;  //最大重发次数
    HAL_StatusTypeDef status = HAL_UART_Transmit(huart, pData, Size, Timeout);
    while (cnt-- && status != HAL_OK) {
        status = HAL_UART_Transmit(huart, pData, Size, Timeout);
    }

    if (status == HAL_OK) {
        return UART_SEND_OK;
    }
    return UART_SEND_FAIL;
}
