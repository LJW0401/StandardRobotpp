/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       mechanical_arm_5_axis.c/h
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

#include "robot_param.h"

#if (MECHANICAL_ARM_TYPE == PENGUIN_MINI_ARM)
#ifndef MECHANICAL_ARM_5_AXIS_H
#define MECHANICAL_ARM_5_AXIS_H

#include <stdbool.h>

#include "mechanical_arm.h"
#include "motor.h"
#include "pid.h"
#include "remote_control.h"
#include "struct_typedef.h"
/*-------------------- Structural definition --------------------*/

/**
 * @brief 状态、期望和限制值
 */
typedef struct
{
    float position[5];  // (rad) 关节位置
} Values_t;
typedef struct
{
    pid_type_def joint_angle[5];
    pid_type_def joint_speed[5];
} PID_t;

typedef struct
{
    const RC_ctrl_t * rc;      // 遥控器指针
    MechanicalArmMode_e mode;  // 模式

    /*-------------------- Motors --------------------*/
    Motor_s joint_motor[5];
    /*-------------------- Values --------------------*/

    Values_t reference;    // 期望值
    Values_t feedback;     // 状态值
    Values_t upper_limit;  // 上限值
    Values_t lower_limit;  // 下限值

    bool init_completed[5];  // 初始化完成标志

    PID_t pid;  // PID控制器

} MechanicalArm_s;

extern void InitMechanicalArm(void);

extern void SetMechanicalArmMode(void);

extern void MechanicalArmObserver(void);

extern void MechanicalArmReference(void);

extern void MechanicalArmConsole(void);

extern void SendMechanicalArmCmd(void);

#endif  // MECHANICAL_ARM_5_AXIS_H
#endif  /* MECHANICAL_ARM_5_AXIS */
