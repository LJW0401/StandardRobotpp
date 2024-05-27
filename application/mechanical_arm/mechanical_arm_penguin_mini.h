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

#if (MECHANICAL_ARM_TYPE == MECHANICAL_ARM_PENGUIN_MINI_ARM)
#ifndef MECHANICAL_ARM_PENGUIN_MINI_H
#define MECHANICAL_ARM_PENGUIN_MINI_H

#include "mechanical_arm.h"
#include "motor.h"
#include "pid.h"
#include "remote_control.h"
#include "stdbool.h"
#include "struct_typedef.h"

// clang-format off
#define DBUS_ERROR_OFFSET      ((uint8_t)1 << 0)  // dbus错误偏移量
#define JOINT_0_ERROR_OFFSET   ((uint8_t)1 << 1)  // 关节0错误偏移量
#define JOINT_1_ERROR_OFFSET   ((uint8_t)1 << 2)  // 关节1错误偏移量
#define JOINT_2_ERROR_OFFSET   ((uint8_t)1 << 3)  // 关节2错误偏移量
#define JOINT_3_ERROR_OFFSET   ((uint8_t)1 << 4)  // 关节3错误偏移量
#define JOINT_4_ERROR_OFFSET   ((uint8_t)1 << 5)  // 关节4错误偏移量
// clang-format on

/*-------------------- Structural definition --------------------*/

typedef enum {
    MECHANICAL_ARM_ZERO_FORCE,
    MECHANICAL_ARM_INIT,
    MECHANICAL_ARM_SET_ZERO,
    MECHANICAL_ARM_FOLLOW,
    MECHANICAL_ARM_STOP,
} MechanicalArmMode_e;

/**
 * @brief 状态、期望和限制值
 */
typedef struct
{
    float pos[5];  // (rad) 关节位置
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
    uint8_t error_code;        // 错误码

    /*-------------------- Motors --------------------*/
    Motor_s joint_motor[5];
    /*-------------------- Values --------------------*/

    Values_t ref;          // 期望值
    Values_t fdb;          // 状态值
    Values_t upper_limit;  // 上限值
    Values_t lower_limit;  // 下限值

    bool init_completed[5];  // 初始化完成标志

    PID_t pid;  // PID控制器

} MechanicalArm_s;

extern void MechanicalArmInit(void);

extern void MechanicalArmHandleException(void);

extern void MechanicalArmSetMode(void);

extern void MechanicalArmObserver(void);

extern void MechanicalArmReference(void);

extern void MechanicalArmConsole(void);

extern void MechanicalArmSendCmd(void);

#endif  // MECHANICAL_ARM_PENGUIN_MINI_H
#endif  /* MECHANICAL_ARM_PENGUIN_MINI */
