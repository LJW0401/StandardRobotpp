/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       chassis.c/h
  * @brief      底盘控制任务所需要的变量和函数
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

#ifndef CHASSIS_H
#define CHASSIS_H

#include <math.h>

#include "pid.h"

#include "remote_control.h"
#include "motor.h"
#include "struct_typedef.h"
#include "robot_param.h"
#include "IMU_task.h"

typedef enum
{
    CHASSIS_ZERO_FORCE,        // 底盘无力，所有控制量置0
    CHASSIS_FOLLOW_GIMBAL_YAW, // 底盘跟随云台（运动方向为云台坐标系方向，需进行坐标转换）
    CHASSIS_STOP,              // 底盘停止运动
    CHASSIS_FREE,              // 底盘不跟随云台
    CHASSIS_SPIN,              // 底盘小陀螺模式
    CHASSIS_AUTO,              // 底盘自动模式
    CHASSIS_OPEN               // 遥控器的值乘以比例成电流值开环控制
} ChassisMode_e;

typedef struct // 底盘速度向量结构体
{
    float vx; // (m/s) x方向速度
    float vy; // (m/s) y方向速度
    float wz; // (rad/s) 旋转速度
} ChassisSpeedVector_t;

/*-------------------- Structural definition --------------------*/
#if (CHASSIS_TYPE == CHASSIS_MECANUM_WHEEL)
#elif (CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)
#elif (CHASSIS_TYPE == CHASSIS_STEERING_WHEEL)
#elif (CHASSIS_TYPE == CHASSIS_BALANCE)
typedef struct
{
    float angle, length;   // rad, m
    float dAngle, dLength; // rad/s, m/s
    float ddLength;        // m/s^2
} LegPos_t;

/**
 * @brief 状态、期望和限制值
 */
typedef struct
{
    /* 用于LQR控制器的状态向量
     * 0-theta
     * 1-theta_dot
     * 2-x
     * 3-x_dot
     * 4-phi
     * 5-phi_dot*/
    float x[6];
    float roll;
    float yaw;
    float leg_length;
    LegPos_t leg_pos[2]; // 0-左腿，1-右腿
    ChassisSpeedVector_t speed_vector;
} Values_t;

typedef struct
{
    pid_type_def yaw_pid_angle;
    pid_type_def yaw_pid_velocity;
} PID_t;
#endif

/**
 * @brief  底盘数据结构体
 * @note   底盘坐标使用右手系，前进方向为x轴，左方向为y轴，上方向为z轴
 */
typedef struct
{
    const RC_ctrl_t *rc; // 底盘使用的遥控器指针
    ChassisMode_e mode;  // 底盘模式

    /*-------------------- Motors --------------------*/
    DJI_Motor_s *yaw_motor; // yaw轴电机
#if (CHASSIS_TYPE == CHASSIS_MECANUM_WHEEL)
    DJI_Motor_s *motor[4]; // 底盘电机
#elif (CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)
    DJI_Motor_s *motor[4]; // 底盘电机
#elif (CHASSIS_TYPE == CHASSIS_STEERING_WHEEL)
    // 舵轮底盘有4个舵轮电机和4个驱动轮电机
    DJI_Motor_s *servo_motor[4]; // 舵机电机
    DJI_Motor_s *wheel_motor[4]; // 驱动轮电机
#elif (CHASSIS_TYPE == CHASSIS_BALANCE)
    // 平衡底盘有2个驱动轮电机和4个关节电机
    DJI_Motor_s *joint_motor[4]; // 关节电机
    DJI_Motor_s *wheel_motor[2]; // 驱动轮电机
#endif

    /*-------------------- Values --------------------*/
#if (CHASSIS_TYPE == CHASSIS_BALANCE)
    ImuData_t *imu; // 底盘使用的IMU数据
#endif

    Values_t reference;   // 期望值
    Values_t feedback;    // 状态值
    Values_t upper_limit; // 上限值
    Values_t lower_limit; // 下限值

    PID_t pid; // PID控制器

    float dyaw;       // (rad)当前位置与云台中值角度差（用于坐标转换）
    uint16_t yaw_mid; // (编码角)云台中值角度
} Chassis_s;

extern Chassis_s chassis;

/*-------------------- Init --------------------*/
#if (CHASSIS_TYPE == CHASSIS_MECANUM_WHEEL)
#elif (CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)
#elif (CHASSIS_TYPE == CHASSIS_STEERING_WHEEL)
#elif (CHASSIS_TYPE == CHASSIS_BALANCE)
void InitBalanceChassisMotor(Chassis_s *chassis);
#endif

/*-------------------- Observe --------------------*/
#if (CHASSIS_TYPE == CHASSIS_MECANUM_WHEEL)
#elif (CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)
#elif (CHASSIS_TYPE == CHASSIS_STEERING_WHEEL)
#elif (CHASSIS_TYPE == CHASSIS_BALANCE)
#endif

/*-------------------- Reference --------------------*/
#if (CHASSIS_TYPE == CHASSIS_MECANUM_WHEEL)
#elif (CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)
#elif (CHASSIS_TYPE == CHASSIS_STEERING_WHEEL)
#elif (CHASSIS_TYPE == CHASSIS_BALANCE)
#endif

/*-------------------- Cmd --------------------*/
#if (CHASSIS_TYPE == CHASSIS_MECANUM_WHEEL)
#elif (CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)
#elif (CHASSIS_TYPE == CHASSIS_STEERING_WHEEL)
#elif (CHASSIS_TYPE == CHASSIS_BALANCE)
void SendBalanceChassisCmd(Chassis_s *chassis);
#endif

#endif // CHASSIS_H
