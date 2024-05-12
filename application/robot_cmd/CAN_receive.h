/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       can_receive.c/h
  * @brief      CAN中断接收函数，接收电机数据.
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
    DJI_M5_ID  = 0x205,   // 3508/2006电机ID (/6020电机ID 不建议使用)
    DJI_M6_ID  = 0x206,   // 3508/2006电机ID (/6020电机ID 不建议使用)
    DJI_M7_ID  = 0x207,   // 3508/2006电机ID (/6020电机ID 不建议使用)
    DJI_M8_ID  = 0x208,   // 3508/2006电机ID (/6020电机ID 不建议使用)
    DJI_M9_ID  = 0x209,   // 6020电机ID
    DJI_M10_ID = 0x20A,  // 6020电机ID
    DJI_M11_ID = 0x20B,  // 6020电机ID
} DJI_Motor_ID;
// clang-format on

extern const DjiMotorMeasure_t * GetDjiMotorMeasurePoint(uint8_t can, uint8_t i);

extern CybergearModeState_e GetCybergearModeState(Motor_s * p_motor);

extern void GetMotorMeasure(Motor_s * p_motor);

#endif
