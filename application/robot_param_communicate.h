/**
  * @file       robot_param_balanced_infantry.h
  * @brief      这里是平衡步兵机器人参数配置文件，包括物理参数、PID参数等
  */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H
#include "robot_typedef.h"

// clang-format off
#define CHASSIS_TYPE         CHASSIS_NONE         // 选择底盘类型
#define GIMBAL_TYPE          GIMBAL_NONE          // 选择云台类型
#define SHOOT_TYPE           SHOOT_NONE           // 选择发射机构类型
#define MECHANICAL_ARM_TYPE  MECHANICAL_ARM_NONE  // 选择机械臂类型
#define CONTROL_TYPE         CHASSIS_AND_GIMBAL   // 选择控制类型
// clang-format on
#endif /* INCLUDED_ROBOT_PARAM_H */
