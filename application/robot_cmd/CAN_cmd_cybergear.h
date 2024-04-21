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

extern void CybergearInit(CyberGear_s * hmotor, uint8_t can, uint8_t motor_id);

/*-------------------- 按照小米电机文档写的各种通信类型 --------------------*/
extern void CybergearControl(
    CyberGear_s * hmotor, float torque, float MechPosition, float velocity, float kp, float kd);

extern void CybergearEnable(CyberGear_s * hmotor);

extern void CybergearStop(CyberGear_s * hmotor);

extern void CybergearSetMechPositionToZero(CyberGear_s * hmotor);

/*-------------------- 封装的一些控制函数 --------------------*/

extern void CybergearTorqueControl(CyberGear_s * hmotor, float torque);

extern void CybergearPositionControl(CyberGear_s * hmotor, float position, float kp, float kd);

extern void CybergearVelocityControl(CyberGear_s * hmotor, float velocity, float kd);

#endif  //CAN_CMD_CYBERGEAR_H
