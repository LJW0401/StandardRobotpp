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

// 控制类型（板间通信时用到）
#define CHASSIS_ONLY 0       // 只控制底盘
#define GIMBAL_ONLY 1        // 只控制云台
#define CHASSIS_AND_GIMBAL 2 // 控制底盘和云台

// 选择底盘类型
#define CHASSIS_TYPE CHASSIS_OMNI_WHEEL
// 选择控制类型
#define CONTROL_TYPE CHASSIS_AND_GIMBAL

// 不同底盘下需要不同的底盘参数
#if (CHASSIS_TYPE == CHASSIS_MECANUM_WHEEL)
#define WZ_SET_SCALE 0.1f               //
#define MOTOR_DISTANCE_TO_CENTER 0.235f // (m)电机到底盘中心距离

#elif (CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)
#define LENGTH_L 0.545f   //(m)底盘对角线长度
#define WHEEL_RADIUS 0.1f //(m)轮子半径
typedef enum
{
    // 底盘CAN1
    WHEEL1 = 0,
    WHEEL2 = 1,
    WHEEL3 = 2,
    WHEEL4 = 3,
    // 云台CAN2
    YAW = 1,
    PITCH = 2,
    TRIGGER = 7,
    FRIC1 = 0,
    FRIC2 = 1,
    FRIC3 = 2,
    FRIC4 = 3
} MotorId_e;

#elif (CHASSIS_TYPE == CHASSIS_STEERING_WHEEL)
#define WHEEL_NUM 4

#elif (CHASSIS_TYPE == CHASSIS_BALANCE)
#define WHEEL_NUM 2

#else
#error "Please select a valid chassis type"
#endif

// 云台模式
#define GIMBAL_MODE_GYRO 0    // 陀螺仪角度控制
#define GIMBAL_MODE_ENCODER 1 // 编码器角度控制
#define GIMBAL_MODE_AUTO 2    // 云台自动控制

// 通用系数
#define RC_TO_ONE 0.0015151515151515f // 遥控器通道值归一化系数

#endif /* ROBOT_PARAM_H */
