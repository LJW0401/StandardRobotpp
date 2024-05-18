/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       chassis_balance.c/h
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
#ifndef CHASSIS_BALANCE_H
#define CHASSIS_BALANCE_H

#include "robot_param.h"

#if (CHASSIS_TYPE == CHASSIS_BALANCE)
#include "IMU_task.h"
#include "chassis.h"
#include "math.h"
#include "motor.h"
#include "pid.h"
#include "remote_control.h"
#include "struct_typedef.h"

// clang-format off
#define JOINT_ERROR_OFFSET   ((uint8_t)1 << 0)  // 关节电机错误偏移量
#define WHEEL_ERROR_OFFSET   ((uint8_t)1 << 1)  // 驱动轮电机错误偏移量
#define DBUS_ERROR_OFFSET          ((uint8_t)1 << 2)  // dbus错误偏移量
// clang-format on

/*-------------------- Structural definition --------------------*/

typedef struct
{
    float angle;     // rad
    float length;    // m
    float dAngle;    // rad/s
    float dLength;   // m/s
    float ddLength;  // m/s^2

    float last_dLength;  // m/s
} LegPos_t;

typedef struct
{
    float yaw, pitch, roll;                             // rad
    float yaw_velocity, pitch_velocity, roll_velocity;  // rad/s
    float xAccel, yAccel, zAccel;                       // m/s^2
} Imu_t;

/**
 * @brief      比例系数结构体
 * @note       比例系数，用于手动优化控制效果
 */
typedef struct
{
    float k[2][6];
    float Tp;
    float T;
    float length;
} Ratio_t;

/**
 * @brief 状态、期望和限制值
 */
typedef struct
{
    float theta;
    float theta_dot;
    float x;
    float x_dot;
    float phi;
    float phi_dot;

    float speed_integral;
    float roll;
    float roll_velocity;
    float yaw;
    float yaw_velocity;

    LegPos_t leg_l;
    LegPos_t leg_right;
    ChassisSpeedVector_t speed_vector;
} Values_t;

typedef struct
{
    pid_type_def yaw_angle;
    pid_type_def yaw_velocity;

    pid_type_def roll_angle;
    pid_type_def roll_velocity;

    pid_type_def leg_length_left_length;
    pid_type_def leg_length_left_speed;

    pid_type_def leg_length_right_length;
    pid_type_def leg_length_right_speed;

    pid_type_def leg_angle_angle;
} PID_t;

/**
 * @brief  底盘数据结构体
 * @note   底盘坐标使用右手系，前进方向为x轴，左方向为y轴，上方向为z轴
 */
typedef struct
{
    const RC_ctrl_t * rc;  // 底盘使用的遥控器指针
    ChassisMode_e mode;    // 底盘模式
    ChassisState_e state;  // 底盘状态
    uint8_t error_code;    // 底盘错误代码

    /*-------------------- Motors --------------------*/
    // 平衡底盘有2个驱动轮电机和4个关节电机
    Motor_s joint_motor[4];
    Motor_s wheel_motor[2];  // 驱动轮电机 0-左轮，1-右轮
    /*-------------------- Values --------------------*/
    Imu_t imu;  // (feedback)底盘使用的IMU数据

    Values_t ref;    // 期望值
    Values_t fdb;     // 状态值
    Values_t upper_limit;  // 上限值
    Values_t lower_limit;  // 下限值

    PID_t pid;  // PID控制器

    Ratio_t ratio;  // 比例系数

    float dyaw;  // (rad)(feedback)当前位置与云台中值角度差（用于坐标转换）
    uint16_t yaw_mid;  // (ecd)(preset)云台中值角度
} Chassis_s;

extern void ChassisInit(void);

extern void ChassisHandleException(void);

extern void ChassisSetMode(void);

extern void ChassisObserver(void);

extern void ChassisReference(void);

extern void ChassisConsole(void);

extern void ChassisSendCmd(void);

#endif /* CHASSIS_BALANCE */
#endif /* CHASSIS_BALANCE_H */
