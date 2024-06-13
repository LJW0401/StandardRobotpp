/**
 * @brief uart板级支持包
 * @author Penguin
 */
#include "bsp_uart.h"

#include "CRC8_CRC16.h"
#include "fifo.h"

/**
 * @brief 使用uart发送数据
 * @param huart 
 * @param pData 
 * @param Size 
 * @param Timeout 
 * @return 
 */
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

#define UART2_RX_BUF_LENGHT 512
#define UART2_FIFO_BUF_LENGTH 1024

uint8_t  UART2_BUFFER[2][UART2_RX_BUF_LENGHT];
fifo_s_t UART2_FIFO;
uint8_t  UART2_FIFO_BUFFER[UART2_FIFO_BUF_LENGTH];
// unpack_data_t referee_unpack_obj;

void USART1_IRQHandler(void)
{
    static volatile uint8_t res;
    if (USART1->SR & UART_FLAG_IDLE) {
        __HAL_UART_CLEAR_PEFLAG(&huart1);

        static uint16_t this_time_rx_len = 0;

        if ((huart1.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET) {
            __HAL_DMA_DISABLE(huart1.hdmarx);
            this_time_rx_len = UART2_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
            __HAL_DMA_SET_COUNTER(huart1.hdmarx, UART2_RX_BUF_LENGHT);
            huart1.hdmarx->Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(huart1.hdmarx);
            fifo_s_puts(&UART2_FIFO, (char *)UART2_BUFFER[0], this_time_rx_len);
            // detect_hook(REFEREE_TOE);
        } else {
            __HAL_DMA_DISABLE(huart1.hdmarx);
            this_time_rx_len = UART2_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
            __HAL_DMA_SET_COUNTER(huart1.hdmarx, UART2_RX_BUF_LENGHT);
            huart1.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(huart1.hdmarx);
            fifo_s_puts(&UART2_FIFO, (char *)UART2_BUFFER[1], this_time_rx_len);
            // detect_hook(REFEREE_TOE);
        }
    }
}
