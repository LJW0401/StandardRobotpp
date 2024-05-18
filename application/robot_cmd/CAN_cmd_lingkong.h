/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       can_cmd_lingkong.c/h
  * @brief      CAN发送函数，通过CAN信号控制瓴控电机 9025.
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     May-16-2024     Penguin         1. 完成。
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */
#ifndef CAN_CMD_LINGKONG_H
#define CAN_CMD_LINGKONG_H

#include "motor.h"

void LkDisableMotor(Motor_s * p_motor);
void LkStopMotor(Motor_s * p_motor);
void LkEnableMotor(Motor_s * p_motor);
void LkSingleTorqueControl(Motor_s * p_motor);
void LkMultipleTorqueControl(
    Motor_s * p_motor_1, Motor_s * p_motor_2, Motor_s * p_motor_3, Motor_s * p_motor_4);

#endif /* CAN_CMD_LINGKONG_H */
/************************ END OF FILE ************************/
