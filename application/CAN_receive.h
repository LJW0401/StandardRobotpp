/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       支持DJI电机 GM3508 GM2006 GM6020
  *         未来支持小米电机 Cybergear
  *         未来支持达妙电机 DM8009
  * @history
  *  Version    Date            Author          Modification
  *  V2.0.0     Mar-27-2024     Penguin         1. 添加CAN发送函数和新的电机控制函数，解码中将CAN1 CAN2分开。
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "main.h"

#define CAN_1 hcan1
#define CAN_2 hcan2

/*DJI电机用相关参数定义*/
typedef enum
{
    DJI_200 = 0x200, // 用于3508,2006的电流控制(ID 1~4)
    DJI_1FF = 0x1FF, // 用于3508,2006的电流控制(ID 5~8);6020的电压控制(ID 1~4)
    DJI_2FF = 0x2FF, // 用于6020的电压控制(ID 5~7)
    DJI_1FE = 0x1FE, // 用于6020的电流控制(ID 1~4)
    DJI_2FE = 0x2FE, // 用于6020的电流控制(ID 5~7)
} DJI_Std_ID;

typedef enum
{
    DJI_M1_ID = 0x201,  // 3508/2006电机ID
    DJI_M2_ID = 0x202,  // 3508/2006电机ID
    DJI_M3_ID = 0x203,  // 3508/2006电机ID
    DJI_M4_ID = 0x204,  // 3508/2006电机ID
    DJI_M5_ID = 0x205,  // 3508/2006电机ID (/6020电机ID 不建议使用)
    DJI_M6_ID = 0x206,  // 3508/2006电机ID (/6020电机ID 不建议使用)
    DJI_M7_ID = 0x207,  // 3508/2006电机ID (/6020电机ID 不建议使用)
    DJI_M8_ID = 0x208,  // 3508/2006电机ID (/6020电机ID 不建议使用)
    DJI_M9_ID = 0x209,  // 6020电机ID
    DJI_M10_ID = 0x20A, // 6020电机ID
    DJI_M11_ID = 0x20B, // 6020电机ID
} DJI_Motor_ID;

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} DJI_Motor_Measure_Data_t;

typedef struct
{
    CAN_HandleTypeDef *CAN;
    DJI_Std_ID std_id;
    CAN_TxHeaderTypeDef tx_message;
    uint8_t can_send_data[8];
} DJI_Motor_Send_Data_s;

void CAN_CmdDJIMotor(DJI_Motor_Send_Data_s *DJI_Motor_Send_Data, int16_t curr_1, int16_t curr_2, int16_t curr_3, int16_t curr_4);

// void CAN_CmdDJIMotor_200(CAN_HandleTypeDef *CAN, int16_t curr_1, int16_t curr_2, int16_t curr_3, int16_t curr_4);
// void CAN_CmdDJIMotor_1FF(CAN_HandleTypeDef *CAN, int16_t curr_5, int16_t curr_6, int16_t curr_7, int16_t curr_8);
// void CAN_CmdDJIMotor_2FF(CAN_HandleTypeDef *CAN, int16_t volt_5, int16_t volt_6, int16_t volt_7);

/*小米电机用相关参数定义*/

/*达妙电机用相关参数定义*/

#endif
