/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       chassis.c/h
  * @brief      底盘部分通用变量和函数的定义
  * @note       将通用内容放在chassis.c中，避免chassis_task.c和chassis_behaviour.c的循环引用
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024      Penguin          1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#ifndef CHASSIS_H
#define CHASSIS_H

#include "remote_control.h"
#include "motor.h"
#include "struct_typedef.h"
#include "robot_param.h"

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

typedef struct // 底盘IMU数据
{
    float yaw, pitch, roll;          // rad
    float yawSpd, pitchSpd, rollSpd; // rad/s
    float xAccel, yAccel, zAccel;    // m/s^2
} ChassisImuData_t;

typedef struct // 底盘速度向量结构体
{
    float vx; // (m/s) x方向速度
    float vy; // (m/s) y方向速度
    float wz; // (rad/s) 旋转速度
} ChassisSpeedVector_t;

/**
 * @brief  底盘数据结构体
 * @note   底盘坐标使用右手系，前进方向为x轴，左方向为y轴，上方向为z轴
 */
typedef struct
{
    const RC_ctrl_t *rc;  // 底盘使用的遥控器指针
    ChassisImuData_t imu; // 底盘使用的IMU数据

    ChassisMode_e mode; // 底盘模式

#if (CHASSIS_TYPE == CHASSIS_BALANCE)          // 平衡底盘有2个驱动轮电机和4个关节电机
    DJI_Motor_s joint_motor[4];                // 关节电机
    DJI_Motor_s wheel_motor[2];                // 驱动轮电机
#elif (CHASSIS_TYPE == CHASSIS_STEERING_WHEEL) // 舵轮底盘有4个舵轮电机和4个驱动轮电机
    DJI_Motor_s servo_motor[4]; // 舵机电机
    DJI_Motor_s wheel_motor[4]; // 驱动轮电机
#else
    DJI_Motor_s motor[4]; // 底盘电机
#endif

    DJI_Motor_s yaw_motor[4]; // yaw轴电机

    ChassisSpeedVector_t speed_vector_set; // 底盘速度向量设置值
    ChassisSpeedVector_t speed_vector;     // 底盘速度向量实际值（需要通过速度解算由轮子的速度得到）
    ChassisSpeedVector_t speed_vector_max; // 底盘速度向量最大值

    float dyaw;    // (rad)当前位置与云台中值角度差（用于坐标转换）
    float yaw_mid; // (rad)云台中值角度
} Chassis_s;

extern Chassis_s chassis;

#if (CHASSIS_TYPE == CHASSIS_MECANUM_WHEEL)
#elif (CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)
#elif (CHASSIS_TYPE == CHASSIS_STEERING_WHEEL)
#elif (CHASSIS_TYPE == CHASSIS_BALANCE)
void SendJointMotorCmd(Chassis_s *chassis);
void SendWheelMotorCmd(Chassis_s *chassis);

#endif

#endif // CHASSIS_H
