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

#ifndef __SEND_DATA_TASK__
#define __SEND_DATA_TASK__

#include "stm32f4xx_hal.h"
#include "referee.h"

#define CONTROLLER_ENGINEERING ((uint8_t)0x01)  // 工程机器人的自定义控制器
#define CONTROLLER_INFANTRY ((uint8_t)0x02)     // 步兵机器人的自定义控制器

#define FRAME_HEADER_LENGTH 5  // 帧头数据长度
#define CMD_ID_LENGTH 2        // 命令码ID数据长度
#define DATA_LENGTH 30         // 数据段长度
#define FRAME_TAIL_LENGTH 2    // 帧尾数据长度

#define DATA_FRAME_LENGTH \
    (FRAME_HEADER_LENGTH + CMD_ID_LENGTH + DATA_LENGTH + FRAME_TAIL_LENGTH)  // 整个数据帧的长度

#define CONTROLLER_CMD_ID 0x0302  // 自定义控制器命令码

typedef __packed struct
{
    __packed struct
    {
        uint8_t sof;               // 起始字节，固定值为0xA5
        uint16_t data_length;      // 数据帧中data的长度
        uint8_t seq;               // 包序号
        uint8_t crc8;              // 帧头CRC8校验
    } frame_header;                // 帧头
    __packed uint16_t cmd_id;      // 命令码
    __packed uint8_t data[30];     // 自定义控制器的数据帧
    __packed uint16_t frame_tail;  // 帧尾CRC16校验
} Controller_t;                    // 自定义控制器数据包

typedef __packed struct __EngineerCustomController
{
    float yaw;
    float big_arm_pitch;
    float small_arm_pitch;
    float small_arm_roll;
} EngineerCustomControllerData_t;

extern void SendDataToPC(uint8_t * data);

extern void EngineeringCustomControllerRxDecode(
    EngineerCustomControllerData_t * engineer_custom_controller_data);

#endif
/************************ END OF FILE ************************/
