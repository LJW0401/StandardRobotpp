/**
  * @file       robot_param_balanced_infantry.h
  * @brief      这里是平衡步兵机器人参数配置文件，包括物理参数、PID参数等
  */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H
#include "robot_typedef.h"

#define CHASSIS_TYPE CHASSIS_BALANCE             // 选择底盘类型
#define GIMBAL_TYPE GIMBAL_NONE                  // 选择云台类型
#define SHOOT_TYPE SHOOT_NONE                    // 选择发射机构类型
#define MECHANICAL_ARM_TYPE MECHANICAL_ARM_NONE  // 选择机械臂类型
#define CONTROL_TYPE CHASSIS_AND_GIMBAL          // 选择控制类型

// clang-format off
/*-------------------- Chassis --------------------*/
// motor parameters ---------------------
#define JOINT_CAN 1
#define WHEEL_CAN 2

//physical parameters ---------------------
#define WHEEL_RADIUS 0.106f  //(m)轮子半径
//upper_limit parameters ---------------------
#define MAX_THETA      1.0f
#define MAX_THETA_DOT  2.0f
#define MAX_X          1.0f
#define MAX_X_DOT      10.0f
#define MAX_PHI        1.0f
#define MAX_PHI_DOT    2.0f

#define MAX_SPEED_INTEGRAL  0.5f
#define MAX_ROLL            1.0f
#define MAX_ROLL_VELOCITY   1.0f
#define MAX_YAW             M_PI
#define MAX_YAW_VELOCITY    3.0f

#define MAX_LEG_LENGTH       0.35f
#define MAX_LEG_ANGLE        M_PI_2 + 0.1f
#define MAX_SPEED_VECTOR_VX  5.0f
#define MAX_SPEED_VECTOR_VY  5.0f
#define MAX_SPEED_VECTOR_WZ  1.0f

//lower_limit parameters ---------------------
#define MIN_THETA      -MAX_THETA
#define MIN_THETA_DOT  -MAX_THETA_DOT
#define MIN_X          -MAX_X
#define MIN_X_DOT      -MAX_X_DOT
#define MIN_PHI        -MAX_PHI
#define MIN_PHI_DOT    -MAX_PHI_DOT

#define MIN_SPEED_INTEGRAL  -MAX_SPEED_INTEGRAL
#define MIN_ROLL            -MAX_ROLL
#define MIN_ROLL_VELOCITY   -MAX_ROLL_VELOCITY
#define MIN_YAW             -MAX_YAW
#define MIN_YAW_VELOCITY    -MAX_YAW_VELOCITY

#define MIN_LEG_LENGTH        0.11f
#define MIN_LEG_ANGLE         M_PI_2 - 0.1f
#define MIN_SPEED_VECTOR_VX  -MAX_SPEED_VECTOR_VX
#define MIN_SPEED_VECTOR_VY  -MAX_SPEED_VECTOR_VY
#define MIN_SPEED_VECTOR_WZ  -MAX_SPEED_VECTOR_WZ



//PID parameters ---------------------
//yaw轴跟踪角度环PID参数
#define KP_CHASSIS_YAW_ANGLE 0.0f
#define KI_CHASSIS_YAW_ANGLE 0.0f
#define KD_CHASSIS_YAW_ANGLE 0.0f
#define MAX_IOUT_CHASSIS_YAW_ANGLE 0.0f
#define MAX_OUT_CHASSIS_YAW_ANGLE 0.0f

//yaw轴跟踪速度环PID参数
#define KP_CHASSIS_YAW_VELOCITY 0.0f
#define KI_CHASSIS_YAW_VELOCITY 0.0f
#define KD_CHASSIS_YAW_VELOCITY 0.0f
#define MAX_IOUT_CHASSIS_YAW_VELOCITY 0.0f
#define MAX_OUT_CHASSIS_YAW_VELOCITY 0.0f

//roll轴跟踪角度环PID参数
#define KP_CHASSIS_ROLL_ANGLE 0.0f
#define KI_CHASSIS_ROLL_ANGLE 0.0f
#define KD_CHASSIS_ROLL_ANGLE 0.0f
#define MAX_IOUT_CHASSIS_ROLL_ANGLE 0.0f
#define MAX_OUT_CHASSIS_ROLL_ANGLE 0.0f

//roll轴跟踪速度环PID参数
#define KP_CHASSIS_ROLL_VELOCITY 0.0f
#define KI_CHASSIS_ROLL_VELOCITY 0.0f
#define KD_CHASSIS_ROLL_VELOCITY 0.0f
#define MAX_IOUT_CHASSIS_ROLL_VELOCITY 0.0f
#define MAX_OUT_CHASSIS_ROLL_VELOCITY 0.0f

// 腿长跟踪长度环PID参数
#define KP_CHASSIS_LEG_LENGTH_LENGTH 0.0f
#define KI_CHASSIS_LEG_LENGTH_LENGTH 0.0f
#define KD_CHASSIS_LEG_LENGTH_LENGTH 0.0f
#define MAX_IOUT_CHASSIS_LEG_LENGTH_LENGTH 0.0f
#define MAX_OUT_CHASSIS_LEG_LENGTH_LENGTH 0.0f

// 腿长跟踪速度环PID参数
#define KP_CHASSIS_LEG_LENGTH_SPEED 0.0f
#define KI_CHASSIS_LEG_LENGTH_SPEED 0.0f
#define KD_CHASSIS_LEG_LENGTH_SPEED 0.0f
#define MAX_IOUT_CHASSIS_LEG_LENGTH_SPEED 0.0f
#define MAX_OUT_CHASSIS_LEG_LENGTH_SPEED 0.0f

// 腿角控制角度环PID参数
#define KP_CHASSIS_LEG_ANGLE_ANGLE 0.0f
#define KI_CHASSIS_LEG_ANGLE_ANGLE 0.0f
#define KD_CHASSIS_LEG_ANGLE_ANGLE 0.0f
#define MAX_IOUT_CHASSIS_LEG_ANGLE_ANGLE 0.0f
#define MAX_OUT_CHASSIS_LEG_ANGLE_ANGLE 0.0f

//other parameters ---------------------
#define LEG_DDLENGTH_LPF_RATIO 0.5f  // 低通滤波系数

/*-------------------- Gimbal --------------------*/
//physical parameters ---------------------
//PID parameters ---------------------

/*-------------------- Shoot --------------------*/
//physical parameters ---------------------
#define FRIC_RADIUS 0.03f  // (m)摩擦轮半径
#define BULLET_NUM 8       // 定义拨弹盘容纳弹丸个数
#define GUN_NUM 1          // 定义枪管个数（一个枪管2个摩擦轮）

//PID parameters ---------------------

// clang-format on
#endif /* INCLUDED_ROBOT_PARAM_H */
