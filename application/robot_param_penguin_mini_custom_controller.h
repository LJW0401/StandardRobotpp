/**
  * @file       robot_param_penguin_mini_custom_controller.h
  * @brief      这里是小企鹅的迷你自定义控制器的参数配置文件，包括物理参数、PID参数等
  */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H
#include "robot_typedef.h"

#define CHASSIS_TYPE CHASSIS_NONE                              // 选择底盘类型
#define GIMBAL_TYPE GIMBAL_NONE                                // 选择云台类型
#define SHOOT_TYPE SHOOT_NONE                                  // 选择发射机构类型
#define MECHANICAL_ARM_TYPE MECHANICAL_ARM_NONE                // 选择机械臂类型
#define CUSTOM_CONTROLLER_TYPE CUSTOM_CONTROLLER_PENGUIN_MINI  // 选择自定义控制器类型
#define CONTROL_TYPE CHASSIS_AND_GIMBAL                        // 选择控制类型

/*-------------------- Custom controller --------------------*/
// clang-format off
#define CTRL_BOARD_ID      1 //机械臂C板id
#define MAIN_BOARD_ID      2 //主C板id
#define BIG_ARM_BOARD_ID   3 //大臂C板id
#define SMALL_ARM_BOARD_ID 4 //小臂C板id
// clang-format on

#endif /* INCLUDED_ROBOT_PARAM_H */
