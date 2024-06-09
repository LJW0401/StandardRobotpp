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
// 底盘任务相关宏定义
#define CHASSIS_TASK_INIT_TIME 357   // 任务开始空闲一段时间
#define CHASSIS_CONTROL_TIME_MS 2    // 底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME 0.002f  // 底盘任务控制间隔 0.002s

// 底盘的遥控器相关宏定义 ---------------------
#define CHASSIS_MODE_CHANNEL 0  // 选择底盘状态 开关通道号
#define CHASSIS_X_CHANNEL 1     // 前后的遥控器通道号码
#define CHASSIS_Y_CHANNEL 0     // 左右的遥控器通道号码
#define CHASSIS_WZ_CHANNEL 0    // 旋转的遥控器通道号码
#define CHASSIS_RC_DEADLINE 5  // 摇杆死区

// deadzone parameters ---------------------
#define WHEEL_DEADZONE 0.02f  // (m/s)轮子速度死区

// motor parameters ---------------------
#define JOINT_CAN 1
#define WHEEL_CAN 2

#define J0_DIRECTION  1
#define J1_DIRECTION  1
#define J2_DIRECTION -1
#define J3_DIRECTION -1

#define W0_DIRECTION  1
#define W1_DIRECTION -1

//physical parameters ---------------------
#define WHEEL_RADIUS         0.106f  //(m)轮子半径
#define WHEEL_START_TORQUE   0.3f  // (Nm)轮子起动力矩
#define J0_ANGLE_OFFSET     -0.19163715f  // (rad)关节0角度偏移量(电机0点到水平线的夹角)
#define J1_ANGLE_OFFSET      M_PI + 0.19163715f  // (rad)关节1角度偏移量(电机0点到水平线的夹角)
#define J2_ANGLE_OFFSET      0.19163715f  // (rad)关节2角度偏移量(电机0点到水平线的夹角)
#define J3_ANGLE_OFFSET     -(M_PI + 0.19163715f)  // (rad)关节3角度偏移量(电机0点到水平线的夹角)

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

#define MAX_J0_ANGLE  1.8f // (rad)关节角度上限
#define MAX_J1_ANGLE  0.0f // (rad)关节角度上限
#define MAX_J2_ANGLE  0.6f // (rad)关节角度上限
#define MAX_J3_ANGLE  1.8f // (rad)关节角度上限

#define MAX_LEG_LENGTH       0.35f
#define MAX_LEG_ANGLE        M_PI_2 + 0.1f
#define MAX_SPEED            5.0f
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

#define MIN_J0_ANGLE -0.6f // (rad)关节角度下限
#define MIN_J1_ANGLE -1.8f // (rad)关节角度下限
#define MIN_J2_ANGLE -1.8f // (rad)关节角度下限
#define MIN_J3_ANGLE  0.0f // (rad)关节角度下限

#define MIN_LEG_LENGTH        0.11f
#define MIN_LEG_ANGLE         M_PI_2 - 0.1f
#define MIN_SPEED            -MAX_SPEED
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
