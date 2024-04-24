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
    MotorInit(&MECHANICAL_ARM.joint_motor[0], 1, 1, CYBERGEAR_MOTOR);
    MotorInit(&MECHANICAL_ARM.joint_motor[1], 2, 1, CYBERGEAR_MOTOR);
    MotorInit(&MECHANICAL_ARM.joint_motor[2], 3, 1, CYBERGEAR_MOTOR);
    MotorInit(&MECHANICAL_ARM.joint_motor[3], 5, 2, DJI_M6020);
    MotorInit(&MECHANICAL_ARM.joint_motor[4], 2, 2, DJI_M3508);
    // CybergearEnable(&MECHANICAL_ARM.joint_motor[0]);
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
    for (uint8_t i = 1; i < 5; i++) {
        GetMotorMeasure(&MECHANICAL_ARM.joint_motor[i]);
    }
    OutputPCData.data_1 = MECHANICAL_ARM.joint_motor[3].w;
    OutputPCData.data_2 = MECHANICAL_ARM.joint_motor[3].temperature;
    OutputPCData.data_3 = MECHANICAL_ARM.joint_motor[3].pos;
    OutputPCData.data_4 = MECHANICAL_ARM.joint_motor[3].current;
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
void MechanicalArmConsole(void)
{
    MECHANICAL_ARM.joint_motor[3].current_set = 1500;
    MECHANICAL_ARM.joint_motor[4].current_set = 1000;
}

/*-------------------- Cmd --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */
void SendMechanicalArmCmd(void)
{
    // CybergearVelocityControl(&MECHANICAL_ARM.joint_motor[0], 1, 0.5);
    CanCmdDjiMotor(2, DJI_2FF, MECHANICAL_ARM.joint_motor[3].current_set, 0, 0, 0);
    CanCmdDjiMotor(2, DJI_200, 0, MECHANICAL_ARM.joint_motor[4].current_set, 0, 0);
}

#endif /* MECHANICAL_ARM_5_AXIS */
