/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       chassis_omni.c/h
  * @brief      平衡底盘控制器。
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

#if (CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)
#ifndef CHASSIS_OMNI_H
#define CHASSIS_OMNI_H

#include <math.h>

#include "IMU_task.h"
#include "chassis.h"
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
    float wheel_speed[4];  // (m/s)轮子速度
    ChassisSpeedVector_t speed_vector;
} Values_t;

typedef struct
{
    pid_type_def wheel_pid_speed[4];

    pid_type_def gimbal_follow_pid_angle;
} PID_t;

/**
 * @brief  底盘数据结构体
 * @note   底盘坐标使用右手系，前进方向为x轴，左方向为y轴，上方向为z轴
 */
typedef struct
{
    const RC_ctrl_t * rc;  // 底盘使用的遥控器指针
    ChassisMode_e mode;    // 底盘模式

    /*-------------------- Motors --------------------*/
    DJI_Motor_s yaw_motor;  // yaw轴电机
    DJI_Motor_s wheel_motor[4];  // 驱动轮电机
    /*-------------------- Values --------------------*/
    ImuData_t * imu;  // (feedback)底盘使用的IMU数据

    Values_t reference;    // 期望值
    Values_t feedback;     // 状态值
    Values_t upper_limit;  // 上限值
    Values_t lower_limit;  // 下限值

    PID_t pid;  // PID控制器

    float dyaw;  // (rad)(feedback)当前位置与云台中值角度差（用于坐标转换）
    uint16_t yaw_mid;  // (ecd)(preset)云台中值角度
} Chassis_s;

extern void InitChassis(void);

extern void SetChassisMode(void);

extern void ChassisObserver(void);

extern void ChassisReference(void);

extern void ChassisConsole(void);

extern void SendChassisCmd(void);

#endif /* CHASSIS_OMNI_H */
#endif /* CHASSIS_OMNI */
