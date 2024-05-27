/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       can_cmd_cybergear.c/h
  * @brief      CAN发送函数，通过CAN信号控制小米电机 Cybergear.
  * @history
  *  Version    Date            Author          Modification
  *  V2.0.0     Apr-19-2024     Penguin         1. 完成。
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#ifndef CAN_CMD_CYBERGEAR_H
#define CAN_CMD_CYBERGEAR_H

#include "motor.h"
#include "stm32f4xx_hal.h"

/*-------------------- 按照小米电机文档写的各种通信类型 --------------------*/
extern void CybergearControl(
    Motor_s * p_motor, float torque, float MechPosition, float velocity, float kp, float kd);

extern void CybergearEnable(Motor_s * p_motor);

extern void CybergearStop(Motor_s * p_motor);

extern void CybergearSetMechPositionToZero(Motor_s * p_motor);

extern void CybergearReadParam(Motor_s* p_motor,uint16_t index);

/*-------------------- 封装的一些控制函数 --------------------*/

extern void CybergearTorqueControl(Motor_s * p_motor);

extern void CybergearPositionControl(Motor_s * p_motor, float kp, float kd);

extern void CybergearVelocityControl(Motor_s * p_motor, float kd);

#endif  //CAN_CMD_CYBERGEAR_H
