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

extern void DmEnable(Motor_s * motor, DmMode_e mode_id);
extern void DmDisable(Motor_s * motor, DmMode_e mode_id);
extern void DmSavePosZero(Motor_s * motor, DmMode_e mode_id);

extern void DmMitCtrlTorque(Motor_s * motor);
extern void DmMitCtrlVelocity(Motor_s * motor, float kd);
extern void DmMitCtrlPosition(Motor_s * motor, float kp, float kd);

extern void DmPosCtrl(Motor_s * motor);
extern void DmSpeedCtrl(Motor_s * motor);
#endif /* CAN_CMD_DAMIAO_H */
/************************ END OF FILE ************************/
