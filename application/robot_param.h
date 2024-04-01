/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       robot_param.h
  * @brief      这里是机器人参数配置文件，包括底盘参数，物理参数等
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-31-2024     Penguin         1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#ifndef ROBOT_PARAM_H
#define ROBOT_PARAM_H

// 可用底盘类型
#define CHASSIS_MECANUM_WHEEL 0  // 麦克纳姆轮底盘
#define CHASSIS_OMNI_WHEEL 1     // 全向轮底盘
#define CHASSIS_STEERING_WHEEL 2 // 舵轮底盘
#define CHASSIS_BALANCE 3        // 平衡底盘

// 选择底盘类型
#define CHASSIS_TYPE CHASSIS_OMNI_WHEEL

// 不同底盘下需要不同的底盘参数
#if (CHASSIS_TYPE == CHASSIS_MECANUM_WHEEL)
#define WZ_SET_SCALE 0.1f               //
#define MOTOR_DISTANCE_TO_CENTER 0.235f // (m)电机到底盘中心距离

#elif (CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)
#define LENGTH_L 0.545f //(m)底盘对角线长度

#elif (CHASSIS_TYPE == CHASSIS_STEERING_WHEEL)
#define WHEEL_NUM 4

#elif (CHASSIS_TYPE == CHASSIS_BALANCE)
#define WHEEL_NUM 2

#else
#error "Please select a valid chassis type"
#endif

// 底盘模式
#define CHASSIS_MODE_FOLLOW_YAW 0 // 云台跟随模式
#define CHASSIS_MODE_FREE 1       // 底盘自由模式
#define CHASSIS_MODE_SPIN 2       // 底盘小陀螺模式
#define CHASSIS_MODE_AUTO 2       // 底盘自动模式

// 云台模式
#define GIMBAL_MODE_GYRO 0    // 陀螺仪角度控制
#define GIMBAL_MODE_ENCODER 1 // 编码器角度控制
#define GIMBAL_MODE_AUTO 2    // 云台自动控制

#endif /* ROBOT_PARAM_H */
