/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       can_cmd_dji.c/h
  * @brief      CAN发送函数，通过CAN信号控制达妙电机 8009.
  * @history
  *  Version    Date            Author          Modification
  *  V2.0.0     May-15-2024     Penguin         1. 完成。
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */
#ifndef CAN_CMD_DAMIAO_H
#define CAN_CMD_DAMIAO_H

#include "motor.h"

#ifndef CAN_N
#define CAN_N
#define CAN_1 hcan1
#define CAN_2 hcan2
#endif

/*-------------------- User functions --------------------*/
void DmEnable(Motor_s * motor);
void DmDisable(Motor_s * motor);
void DmSavePosZero(Motor_s * motor);
void DmMitCtrlTorque(Motor_s * motor);
void DmMitCtrlVelocity(Motor_s * motor, float kd);
void DmMitCtrlPosition(Motor_s * motor, float kp, float kd);


#endif /* CAN_CMD_DAMIAO_H */
/************************ END OF FILE ************************/
