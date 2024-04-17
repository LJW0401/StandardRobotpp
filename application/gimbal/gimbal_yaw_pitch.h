/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       gimbal_yaw_pitch.c/h
  * @brief      yaw_pitch云台控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
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

#include "robot_param.h"
#if (GIMBAL_TYPE == GIMBAL_YAW_PITCH)
#ifndef GIMBAL_YAW_PITCH_H
#define GIMBAL_YAW_PITCH_H
#include "IMU_task.h"
#include "gimbal.h"
#include "motor.h"
#include "pid.h"
#include "remote_control.h"
#include "robot_param.h"
#include "struct_typedef.h"
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

typedef struct
{
    const RC_ctrl_t * rc;  // 遥控器指针
    GimbalMode_e mode;     // 模式

    /*-------------------- Motors --------------------*/

    /*-------------------- Values --------------------*/
    ImuData_t * imu;  // IMU数据

    Values_t reference;    // 期望值
    Values_t feedback;     // 状态值
    Values_t upper_limit;  // 上限值
    Values_t lower_limit;  // 下限值

    PID_t pid;  // PID控制器
} Gimbal_t;

extern void InitGimbal(void);

extern void SetGimbalMode(void);

extern void GimbalObserver(void);

extern void GimbalReference(void);

extern void GimbalConsole(void);

extern void SendGimbalCmd(void);

#endif  // GIMBAL_YAW_PITCH_H
#endif  // GIMBAL_YAW_PITCH
