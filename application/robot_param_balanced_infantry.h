/**
  * @file       robot_param_balanced_infantry.h
  * @brief      这里是平衡步兵机器人参数配置文件，包括物理参数、PID参数等
  */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H

#define CHASSIS_TYPE CHASSIS_BALANCE     // 选择底盘类型
#define GIMBAL_TYPE GIMBAL_YAW_PITCH     // 选择云台类型
#define SHOOT_TYPE SHOOT_FRIC            // 选择发射机构类型
#define CONTROL_TYPE CHASSIS_AND_GIMBAL  // 选择控制类型

// 机器人物理参数
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

/*-------------------- Chassis --------------------*/
//physical parameters ---------------------
#define WHEEL_RADIUS 0.106f  //(m)轮子半径
//upper_limit parameters ---------------------
#define MAX_SPEED_VECTOR_VX 5.0f
#define MAX_SPEED_VECTOR_VY 5.0f
#define MAX_SPEED_VECTOR_WZ 1.0f
#define MAX_ROLL 1.0f
#define MAX_YAW M_PI
#define MAX_LEG_LENGTH 0.35f

#define MAX_X_0 1.0f
#define MAX_X_1 2.0f
#define MAX_X_2 1.0f
#define MAX_X_3 10.0f
#define MAX_X_4 1.0f
#define MAX_X_5 2.0f

//lower_limit parameters ---------------------
#define MIN_SPEED_VECTOR_VX -MAX_SPEED_VECTOR_VX
#define MIN_SPEED_VECTOR_VY -MAX_SPEED_VECTOR_VY
#define MIN_SPEED_VECTOR_WZ -MAX_SPEED_VECTOR_WZ
#define MIN_ROLL -MAX_ROLL
#define MIN_YAW -MAX_YAW
#define MIN_LEG_LENGTH 0.11f

#define MIN_X_0 -MAX_X_0
#define MIN_X_1 -MAX_X_1
#define MIN_X_2 -MAX_X_2
#define MIN_X_3 -MAX_X_3
#define MIN_X_4 -MAX_X_4
#define MIN_X_5 -MAX_X_5

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

/*-------------------- Gimbal --------------------*/
//physical parameters ---------------------
//PID parameters ---------------------

/*-------------------- Shoot --------------------*/
//physical parameters ---------------------
#define FRIC_RADIUS 0.03f  // (m)摩擦轮半径
#define BULLET_NUM 8       // 定义拨弹盘容纳弹丸个数
#define GUN_NUM 1          // 定义枪管个数（一个枪管2个摩擦轮）

//PID parameters ---------------------

#endif /* INCLUDED_ROBOT_PARAM_H */