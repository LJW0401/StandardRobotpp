/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       can_cmd_dji.c/h
  * @brief      CAN发送函数，通过CAN信号控制电机.
  * @note       支持DJI电机 GM3508 GM2006 GM6020
  *         未来支持小米电机 Cybergear
  *         未来支持达妙电机 DM8009
  * @history
  *  Version    Date            Author          Modification
  *  V2.0.0     Mar-27-2024     Penguin         1. 完成。
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#ifndef CAN_CMD_DJI_H
#define CAN_CMD_DJI_H

#include "stm32f4xx_hal.h"
#include "struct_typedef.h"

#ifndef CAN_N
#define CAN_N
#define CAN_1 hcan1
#define CAN_2 hcan2
#endif

/*DJI电机用相关ID定义*/
typedef enum {
    DJI_200 = 0x200,  // 用于3508,2006的电流控制(ID 1-4)
    DJI_1FF = 0x1FF,  // 用于3508,2006的电流控制(ID 5-8);6020的电压控制(ID 1-4)
    DJI_2FF = 0x2FF,  // 用于6020的电压控制(ID 5-7)
    DJI_1FE = 0x1FE,  // 用于6020的电流控制(ID 1-4)
    DJI_2FE = 0x2FE,  // 用于6020的电流控制(ID 5-7)
} DJI_Std_ID;

typedef struct  // DJI电机发送数据结构体
{
    CAN_HandleTypeDef * CAN;
    DJI_Std_ID std_id;
    CAN_TxHeaderTypeDef tx_message;
    uint8_t can_send_data[8];
} DJI_Motor_Send_Data_s;

void CanCmdDjiMotor(
    uint8_t can, DJI_Std_ID std_id, int16_t curr_1, int16_t curr_2, int16_t curr_3, int16_t curr_4);

#endif  //CAN_CMD_DJI_H
