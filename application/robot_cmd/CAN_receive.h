/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       can_receive.c/h
  * @brief      CAN中断接收函数，接收电机数据.
  * @note       支持DJI电机 GM3508 GM2006 GM6020
  *             支持小米电机 Cybergear
  *             支持达妙电机 DM8009
  *             支持瓴控电机 MF9025
  * @history
  *  Version    Date            Author          Modification
  *  V2.0.0     Mar-27-2024     Penguin         1. 添加CAN发送函数和新的电机控制函数，解码中将CAN1 CAN2分开。
  *  V2.1.0     Mar-20-2024     Penguin         1. 添加DM电机的适配
  *  V2.2.0     May-22-2024     Penguin         1. 添加LK电机的适配
  *  V2.3.0     May-22-2024     Penguin         1. 添加板间通信数据解码
  *
  @verbatim
  ==============================================================================
    dm电机设置：
    为了配合本框架，请在使用上位机进行设置时，将dm电机的master id 设置为 slave id + 0x50
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "motor.h"
#include "stm32f4xx_hal.h"

#ifndef CAN_N
#define CAN_N
#define CAN_1 hcan1
#define CAN_2 hcan2
#endif

// clang-format off
/*DJI电机用相关ID定义*/
typedef enum {
    DJI_M1_ID  = 0x201,   // 3508/2006电机ID
    DJI_M2_ID  = 0x202,   // 3508/2006电机ID
    DJI_M3_ID  = 0x203,   // 3508/2006电机ID
    DJI_M4_ID  = 0x204,   // 3508/2006电机ID
    DJI_M5_ID  = 0x205,   // 3508/2006电机ID (/6020电机ID 如分不清关系不建议使用)
    DJI_M6_ID  = 0x206,   // 3508/2006电机ID (/6020电机ID 如分不清关系不建议使用)
    DJI_M7_ID  = 0x207,   // 3508/2006电机ID (/6020电机ID 如分不清关系不建议使用)
    DJI_M8_ID  = 0x208,   // 3508/2006电机ID (/6020电机ID 如分不清关系不建议使用)
    DJI_M9_ID  = 0x209,   // 6020电机ID
    DJI_M10_ID = 0x20A,  // 6020电机ID
    DJI_M11_ID = 0x20B,  // 6020电机ID
} DJI_Motor_ID;

typedef enum __DmMotorType{
    DM_M1_ID = 0x51,
    DM_M2_ID,
    DM_M3_ID,
    DM_M4_ID,
    DM_M5_ID,
    DM_M6_ID,
} DmMotorType_e;

typedef enum __LkMotorType{
    LK_M1_ID = 0x141,
    LK_M2_ID,
    LK_M3_ID,
    LK_M4_ID,
} LkMotorType_e;
// clang-format on

extern const DjiMotorMeasure_t * GetDjiMotorMeasurePoint(uint8_t can, uint8_t i);

extern CybergearModeState_e GetCybergearModeState(Motor_s * p_motor);

extern void GetMotorMeasure(Motor_s * p_motor);

extern uint16_t GetOtherBoardDataUint16(uint8_t data_id, uint8_t data_offset);

#endif
