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
#include "struct_typedef.h"

//导入具体的机器人参数配置文件
#include "robot_param_balanced_infantry.h"
// #include "robot_param_omni_infantry.h"
// #include "robot_param_engineering.h"

// 选择机器人的各种类型
#define __DEBUG 0                  // 调试模式
#define __TUNING 0                 // 调参模式
#define __MUSIC_ON 1               // 开启音乐
#define __TUNING_MODE TUNING_NONE  // 调参模式

/*-------------------- 检测选择是否合法 --------------------*/
#if (                                                                                   \
    (CHASSIS_TYPE != CHASSIS_NONE) && (CHASSIS_TYPE != CHASSIS_MECANUM_WHEEL) &&        \
    (CHASSIS_TYPE != CHASSIS_OMNI_WHEEL) && (CHASSIS_TYPE != CHASSIS_STEERING_WHEEL) && \
    (CHASSIS_TYPE != CHASSIS_BALANCE))
#error "Please select a valid chassis type"
#endif

#if (                                                                     \
    (GIMBAL_TYPE != GIMBAL_NONE) && (GIMBAL_TYPE != GIMBAL_YAW) &&        \
    (GIMBAL_TYPE != GIMBAL_PITCH) && (GIMBAL_TYPE != GIMBAL_YAW_PITCH) && \
    (GIMBAL_TYPE != GIMBAL_YAW_YAW_PITCH) && (GIMBAL_TYPE != GIMBAL_YAW_PITCH_ROLL))
#error "Please select a valid gimbal type"
#endif

#if ((SHOOT_TYPE != SHOOT_NONE) && (SHOOT_TYPE != SHOOT_FRIC) && (SHOOT_TYPE != SHOOT_PNEUMATIC))
#error "Please select a valid shoot type"
#endif

#if (                                                 \
    (MECHANICAL_ARM_TYPE != MECHANICAL_ARM_NONE) &&   \
    (MECHANICAL_ARM_TYPE != MECHANICAL_ARM_3_AXIS) && \
    (MECHANICAL_ARM_TYPE != MECHANICAL_ARM_4_AXIS) && \
    (MECHANICAL_ARM_TYPE != MECHANICAL_ARM_5_AXIS) && \
    (MECHANICAL_ARM_TYPE != MECHANICAL_ARM_6_AXIS) && \
    (MECHANICAL_ARM_TYPE != MECHANICAL_ARM_7_AXIS) && (MECHANICAL_ARM_TYPE != SHOOT_PNEUMATIC))
#error "Please select a valid mechanical arm type"
#endif

#if ((__CONTROL_LINK != DBUS_LINK) && (__CONTROL_LINK != IMAGE_TRANSMISSION_LINK))
#error "Please select a valid control link type"
#endif

#if (                                                                  \
    (CONTROL_TYPE != CHASSIS_ONLY) && (CONTROL_TYPE != GIMBAL_ONLY) && \
    (CONTROL_TYPE != CHASSIS_AND_GIMBAL))
#error "Please select a valid control type"
#endif

#endif /* ROBOT_PARAM_H */
