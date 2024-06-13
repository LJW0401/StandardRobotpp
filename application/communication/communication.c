#include "communication.h"

#include "CRC8_CRC16.h"
#include "bsp_uart.h"
#include "bsp_usart.h"
#include "fifo.h"

#define USART_RX_BUF_LENGHT 512
#define UART2_FIFO_BUF_LENGTH 1024

uint8_t usart1_bufer[2][USART_RX_BUF_LENGHT];

fifo_s_t UART2_FIFO;
uint8_t UART2_FIFO_BUFFER[UART2_FIFO_BUF_LENGTH];
// unpack_data_t referee_unpack_obj;

void __Uart2_Init(void){
    fifo_s_init(&UART2_FIFO, UART2_FIFO_BUFFER, UART2_FIFO_BUF_LENGTH);
    usart1_init(usart1_bufer[0], usart1_bufer[1], USART_RX_BUF_LENGHT);
}

// 4pin Uart口中断处理函数
void __Uart2_IRQHandler(void)
{
    static volatile uint8_t res;
    if (USART1->SR & UART_FLAG_IDLE) {
        __HAL_UART_CLEAR_PEFLAG(&UART2);

        static uint16_t this_time_rx_len = 0;

        if ((UART2.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET) {
            __HAL_DMA_DISABLE(UART2.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(UART2.hdmarx);
            __HAL_DMA_SET_COUNTER(UART2.hdmarx, USART_RX_BUF_LENGHT);
            UART2.hdmarx->Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(UART2.hdmarx);
            fifo_s_puts(&UART2_FIFO, (char *)usart1_bufer[0], this_time_rx_len);
            // detect_hook(REFEREE_TOE);
        } else {
            __HAL_DMA_DISABLE(UART2.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(UART2.hdmarx);
            __HAL_DMA_SET_COUNTER(UART2.hdmarx, USART_RX_BUF_LENGHT);
            UART2.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(UART2.hdmarx);
            fifo_s_puts(&UART2_FIFO, (char *)usart1_bufer[1], this_time_rx_len);
            // detect_hook(REFEREE_TOE);
        }
    }
}
