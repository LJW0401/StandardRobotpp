/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       gimbal.c/h
  * @brief      云台控制任务所需要的变量和函数
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024      Penguin         1. done
  *  V1.0.1     Apr-16-2024     Penguin         1. 完成基本框架
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#ifndef GIMBAL_H
#define GIMBAL_H

#include "pid.h"

#include "remote_control.h"
#include "motor.h"
#include "struct_typedef.h"
#include "robot_param.h"
#include "IMU_task.h"

/**
 * @brief 云台模式
 */
typedef enum
{
    GIMBAL_ZERO_FORCE, // 云台无力，所有控制量置0
    GIMBAL_GYRO,       // 云台陀螺仪控制
    GIMBAL_OPEN,       // 遥控器的值乘以比例成电流值开环控制
} GimbalMode_e;

/*-------------------- Structural definition --------------------*/
#if (GIMBAL_TYPE == GIMBAL_YAW)
#elif (GIMBAL_TYPE == GIMBAL_PITCH)
#elif (GIMBAL_TYPE == GIMBAL_YAW_PITCH)
/**
 * @brief 状态、期望和限制值
 */
typedef struct
{
    float pitch;
    float yaw;
} Values_t;

typedef struct
{
    pid_type_def yaw_pid_angle;
    pid_type_def yaw_pid_velocity;

    pid_type_def pitch_pid_angle;
    pid_type_def pitch_pid_velocity;
} PID_t;
#elif (GIMBAL_TYPE == GIMBAL_YAW_YAW_PITCH)
#elif (GIMBAL_TYPE == GIMBAL_YAW_PITCH_ROLL)
#endif

typedef struct
{
    const RC_ctrl_t *rc; // 遥控器指针
    GimbalMode_e mode;   // 模式

    /*-------------------- Motors --------------------*/

    /*-------------------- Values --------------------*/
    ImuData_t *imu; // IMU数据

    Values_t reference;   // 期望值
    Values_t feedback;    // 状态值
    Values_t upper_limit; // 上限值
    Values_t lower_limit; // 下限值

    PID_t pid; // PID控制器
} Gimbal_t;

extern Gimbal_t gimbal;

/*-------------------- Init --------------------*/
#if (GIMBAL_TYPE == GIMBAL_YAW)
#elif (GIMBAL_TYPE == GIMBAL_PITCH)
#elif (GIMBAL_TYPE == GIMBAL_YAW_PITCH)
#elif (GIMBAL_TYPE == GIMBAL_YAW_YAW_PITCH)
#elif (GIMBAL_TYPE == GIMBAL_YAW_PITCH_ROLL)
#endif

/*-------------------- Observe --------------------*/
#if (GIMBAL_TYPE == GIMBAL_YAW)
#elif (GIMBAL_TYPE == GIMBAL_PITCH)
#elif (GIMBAL_TYPE == GIMBAL_YAW_PITCH)
#elif (GIMBAL_TYPE == GIMBAL_YAW_YAW_PITCH)
#elif (GIMBAL_TYPE == GIMBAL_YAW_PITCH_ROLL)
#endif

/*-------------------- Reference --------------------*/
#if (GIMBAL_TYPE == GIMBAL_YAW)
#elif (GIMBAL_TYPE == GIMBAL_PITCH)
#elif (GIMBAL_TYPE == GIMBAL_YAW_PITCH)
#elif (GIMBAL_TYPE == GIMBAL_YAW_YAW_PITCH)
#elif (GIMBAL_TYPE == GIMBAL_YAW_PITCH_ROLL)
#endif

/*-------------------- Cmd --------------------*/
#if (GIMBAL_TYPE == GIMBAL_YAW)
#elif (GIMBAL_TYPE == GIMBAL_PITCH)
#elif (GIMBAL_TYPE == GIMBAL_YAW_PITCH)
#elif (GIMBAL_TYPE == GIMBAL_YAW_YAW_PITCH)
#elif (GIMBAL_TYPE == GIMBAL_YAW_PITCH_ROLL)
#endif

#endif // GIMBAL_H
