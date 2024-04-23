/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       mechanical_arm.c/h
  * @brief      机械臂功能。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.1     Apr-21-2024     Penguin         1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#include "mechanical_arm_5_axis.h"

#if (MECHANICAL_ARM_TYPE == MECHANICAL_ARM_5_AXIS)
#include "CAN_communication.h"
#include "usb_task.h"

MechanicalArm_s MECHANICAL_ARM;

/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param[in]      none
 * @retval         none
 */
void InitMechanicalArm(void)
{
    MotorInit(&MECHANICAL_ARM.cybergear[0], 1, 1, CYBERGEAR_MOTOR);
    CybergearEnable(&MECHANICAL_ARM.cybergear[0]);
}

/*-------------------- Set mode --------------------*/

/**
 * @brief          设置模式
 * @param[in]      none
 * @retval         none
 */
void SetMechanicalArmMode(void) {}

/*-------------------- Observe --------------------*/

/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void MechanicalArmObserver(void)
{
    GetMotorMeasure(&MECHANICAL_ARM.cybergear[0]);
    OutputPCData.data_1 = MECHANICAL_ARM.cybergear[0].w;
    OutputPCData.data_2 = GetCybergearModeState(&MECHANICAL_ARM.cybergear[0]);
    OutputPCData.data_3 = MECHANICAL_ARM.cybergear[0].pos;
    OutputPCData.data_4 = MECHANICAL_ARM.cybergear[0].T;
}

/*-------------------- Reference --------------------*/

/**
 * @brief          更新目标量
 * @param[in]      none
 * @retval         none
 */
void MechanicalArmReference(void) {}

/*-------------------- Console --------------------*/

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void MechanicalArmConsole(void) {}

/*-------------------- Cmd --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */
void SendMechanicalArmCmd(void) { CybergearVelocityControl(&MECHANICAL_ARM.cybergear[0], 1, 0.5); }

#endif /* MECHANICAL_ARM_5_AXIS */
