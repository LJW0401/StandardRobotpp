/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       custom_controller_connect.c/h
  * @brief      这部分内容负责自定义控制器与电脑的连接
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-30-2024     Penguin         1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#include "custom_controller_connect.h"

#include "CRC8_CRC16.h"
#include "string.h"
#include "usart.h"

/*-------------------- Send --------------------*/

static Controller_t TX_DATA;  // 自定义控制器发送的数据

/**
 * @brief 数据拼接函数，将帧头、命令码、数据段、帧尾头拼接成一个数组
 * @param data 数据段的数组指针
 * @param data_lenth 数据段长度
 */
static void DataConcatenat(uint8_t * data, uint16_t data_lenth)
{
    static uint8_t seq = 0;
    /// 帧头数据
    TX_DATA.frame_header.sof = 0xA5;                // 数据帧起始字节，固定值为 0xA5
    TX_DATA.frame_header.data_length = data_lenth;  // 数据帧中数据段的长度
    TX_DATA.frame_header.seq = seq++;               // 包序号
    append_CRC8_check_sum((uint8_t *)(&TX_DATA.frame_header), 5);  // 添加帧头 CRC8 校验位
    /// 命令码ID
    TX_DATA.cmd_id = CONTROLLER_CMD_ID;
    /// 数据段
    memcpy(TX_DATA.data, data, data_lenth);
    /// 帧尾CRC16，整包校验
    append_CRC16_check_sum((uint8_t *)(&TX_DATA), DATA_FRAME_LENGTH);
}

/**
 * @brief      发送数据到电脑
 * @param[in]  data 自定义数据段（最大30字节）
 */
void SendDataToPC(uint8_t * data)
{
    DataConcatenat(data, DATA_LENGTH);
    HAL_UART_Transmit(&huart1, (uint8_t *)(&TX_DATA), sizeof(TX_DATA), 50);
}

/*-------------------- Receive --------------------*/

void EngineeringCustomControllerRxDecode(
    EngineerCustomControllerData_t * engineer_custom_controller_data)
{
    CustomControllerData_t * custom_controller_data = GetCustomControllerDataPoint();
    memcpy(
        engineer_custom_controller_data, custom_controller_data->data,
        sizeof(EngineerCustomControllerData_t));
}

/************************ END OF FILE ************************/
