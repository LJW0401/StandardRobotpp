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

extern void LkDisable(Motor_s * p_motor);
extern void LkStop(Motor_s * p_motor);
extern void LkEnable(Motor_s * p_motor);
extern void LkSingleSpeedControl(Motor_s * p_motor);
extern void LkSingleTorqueControl(Motor_s * p_motor);
extern void LkMultipleTorqueControl(
    uint8_t can, float torque_1, float torque_2, float torque_3, float torque_4);
extern void LkMultipleIqControl(
    uint8_t can, int16_t iqControl_1, int16_t iqControl_2, int16_t iqControl_3,
    int16_t iqControl_4);
#endif /* CAN_CMD_LINGKONG_H */
/************************ END OF FILE ************************/
