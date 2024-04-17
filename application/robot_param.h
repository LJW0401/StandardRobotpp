/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       robot_param.h
  * @brief      这里是机器人参数配置文件，包括底盘参数，物理参数等
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-31-2024     Penguin         1. done
  *  V1.0.1     Apr-16-2024     Penguin         1. 添加云台和发射机构类型
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#ifndef ROBOT_PARAM_H
#define ROBOT_PARAM_H

#include "robot_typedef.h"

// 选择机器人的各种类型
#define __DEBUG 0                        // 调试模式
#define __TUNING 0                       // 调参模式
#define __CONTROL_LINK DBUS_LINK         // 控制链路
#define CHASSIS_TYPE CHASSIS_BALANCE     // 选择底盘类型
#define GIMBAL_TYPE GIMBAL_YAW_PITCH     // 选择云台类型
#define SHOOT_TYPE SHOOT_FRIC            // 选择发射机构类型
#define CONTROL_TYPE CHASSIS_AND_GIMBAL  // 选择控制类型

// 机器人物理参数
#define GUN_NUM 1     // 定义枪管个数（一个枪管2个摩擦轮）
#define BULLET_NUM 8  // 定义拨弹盘容纳弹丸个数
#define FRIC_RADIUS 0.03f    // (m)摩擦轮半径
#define WHEEL_RADIUS 0.106f  //(m)轮子半径
typedef enum {
    // 底盘CAN1
    WHEEL1 = 0,
    WHEEL2 = 1,
    JOINT1 = 1,
    JOINT2 = 2,
    JOINT3 = 3,
    JOINT4 = 4,
    // 云台CAN2
    YAW = 1,
    PITCH = 2,
    TRIGGER = 7,
    FRIC1 = 0,
    FRIC2 = 1,
} MotorId_e;

#if (                                         \
  (CHASSIS_TYPE != CHASSIS_MECANUM_WHEEL) &&  \
  (CHASSIS_TYPE != CHASSIS_OMNI_WHEEL) &&     \
  (CHASSIS_TYPE != CHASSIS_STEERING_WHEEL) && \
  (CHASSIS_TYPE != CHASSIS_BALANCE))
#error "Please select a valid chassis type"
#endif

#if (                                                                   \
  (GIMBAL_TYPE != GIMBAL_NONE) && (GIMBAL_TYPE != GIMBAL_YAW) &&        \
  (GIMBAL_TYPE != GIMBAL_PITCH) && (GIMBAL_TYPE != GIMBAL_YAW_PITCH) && \
  (GIMBAL_TYPE != GIMBAL_YAW_YAW_PITCH) &&                              \
  (GIMBAL_TYPE != GIMBAL_YAW_PITCH_ROLL))
#error "Please select a valid gimbal type"
#endif

#if (                                                         \
  (SHOOT_TYPE != SHOOT_NONE) && (SHOOT_TYPE != SHOOT_FRIC) && \
  (SHOOT_TYPE != SHOOT_PNEUMATIC))
#error "Please select a valid shoot type"
#endif

#if (                              \
  (__CONTROL_LINK != DBUS_LINK) && \
  (__CONTROL_LINK != IMAGE_TRANSMISSION_LINK))
#error "Please select a valid control link type"
#endif

#if (                                                                \
  (CONTROL_TYPE != CHASSIS_ONLY) && (CONTROL_TYPE != GIMBAL_ONLY) && \
  (CONTROL_TYPE != CHASSIS_AND_GIMBAL))
#error "Please select a valid control type"
#endif

#endif /* ROBOT_PARAM_H */
