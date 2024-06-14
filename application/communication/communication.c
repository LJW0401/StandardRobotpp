/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       communication.h
  * @brief      这里是机器人通信部分
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jun-14-2024     Penguin         1. done
  *
  @verbatim
  ==============================================================================
板间通信的数据包结构如下：
    0x00 header
    0x01 data
    0x03 CRC16
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#include "communication.h"

#include "CRC8_CRC16.h"
#include "bsp_uart.h"
#include "bsp_usart.h"
#include "fifo.h"

#define USART_RX_BUF_LENGHT 512
#define USART1_FIFO_BUF_LENGTH 1024

// send data
BoardCommunicateData_s BOARD_TX_DATA;

// receive data
uint8_t BOARD_RX_DATA[DATA_NUM][DATA_LEN + 1];  //第一位存放数据长度信息

// receive data buffer
uint8_t usart1_buf[2][USART_RX_BUF_LENGHT];
fifo_s_t usart1_fifo;
uint8_t usart1_fifo_buf[USART1_FIFO_BUF_LENGTH];
// unpack_data_t referee_unpack_obj;

// 4pin Uart串口初始化
void Usart1Init(void)
{
    fifo_s_init(&usart1_fifo, usart1_fifo_buf, USART1_FIFO_BUF_LENGTH);
    usart1_init(usart1_buf[0], usart1_buf[1], USART_RX_BUF_LENGHT);
}

// 4pin Uart口中断处理函数
void USART1_IRQHandler(void)
{
    static volatile uint8_t res;
    if (USART1->SR & UART_FLAG_IDLE) {
        __HAL_UART_CLEAR_PEFLAG(&huart1);

        static uint16_t this_time_rx_len = 0;

        if ((huart1.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET) {
            __HAL_DMA_DISABLE(huart1.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
            __HAL_DMA_SET_COUNTER(huart1.hdmarx, USART_RX_BUF_LENGHT);
            huart1.hdmarx->Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(huart1.hdmarx);
            fifo_s_puts(&usart1_fifo, (char *)usart1_buf[0], this_time_rx_len);
            // detect_hook(REFEREE_TOE);
        } else {
            __HAL_DMA_DISABLE(huart1.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
            __HAL_DMA_SET_COUNTER(huart1.hdmarx, USART_RX_BUF_LENGHT);
            huart1.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(huart1.hdmarx);
            fifo_s_puts(&usart1_fifo, (char *)usart1_buf[1], this_time_rx_len);
            // detect_hook(REFEREE_TOE);
        }
    }
}

/**
 * @brief 将数据打包填入发送数据结构体中
 * @param data 数据段的数组指针
 * @param data_lenth 数据段长度
 * @param data_id 数据段id
 */
void DataPack(uint8_t * data, uint8_t data_lenth, uint8_t data_id)
{
    BOARD_TX_DATA.frame_header.sof = 0xA5;        // 数据帧起始字节，固定值为 0xA5
    BOARD_TX_DATA.frame_header.len = data_lenth;  // 数据帧中数据段的长度
    BOARD_TX_DATA.frame_header.id = data_id;
    // 添加帧头 CRC8 校验位
    append_CRC8_check_sum(
        (uint8_t *)(&BOARD_TX_DATA.frame_header), sizeof(BOARD_TX_DATA.frame_header));

    memcpy(BOARD_TX_DATA.data, data, data_lenth);  // 数据段

    // 帧尾CRC16，整包校验
    append_CRC16_check_sum((uint8_t *)(&BOARD_TX_DATA), sizeof(BOARD_TX_DATA));
}

void DataUnpack(void)
{
    uint8_t byte = 0;
    uint8_t frame_header[FRAME_HEADER_LEN] = {0};
    uint8_t received[FRAME_HEADER_LEN + DATA_LEN + 2] = {0};

    while (fifo_s_used(&usart1_fifo)) {
        byte = fifo_s_get(&usart1_fifo);
        if (byte == FRAME_HEADER_SOF) {
            frame_header[0] = byte;
            fifo_s_gets(&usart1_fifo, (char *)(frame_header + 1), FRAME_HEADER_LEN - 1);

            uint8_t header_crc_ok = verify_CRC8_check_sum(frame_header, FRAME_HEADER_LEN);
            if (header_crc_ok) {
                memcpy(received, frame_header, FRAME_HEADER_LEN);  //转移帧头信息
                uint8_t data_len = frame_header[FRAME_HEADER_LEN_OFFEST];
                uint8_t data_id = frame_header[FRAME_HEADER_ID_OFFEST];
                // uint8_t data_type = frame_header[FRAME_HEADER_TYPE_OFFEST];

                fifo_s_gets(&usart1_fifo, (char *)(received + FRAME_HEADER_LEN), DATA_LEN + 2);
                uint16_t crc_ok = verify_CRC16_check_sum(received, sizeof(BoardCommunicateData_s));
                if (crc_ok) {
                    BOARD_RX_DATA[data_id][0] = data_len;
                    memcpy(&BOARD_RX_DATA[data_id][1], received + FRAME_HEADER_LEN, data_len);
                }
            }
        }
    }
}
