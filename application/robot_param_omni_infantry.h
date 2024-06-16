/**
  * @file       robot_param_omni_infantry.h
  * @brief      这里是全向轮步兵机器人参数配置文件，包括物理参数、PID参数等
  */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H
#include "robot_typedef.h"

#define CHASSIS_TYPE CHASSIS_NONE  // 选择底盘类型
#define GIMBAL_TYPE GIMBAL_YAW_PITCH_DIRECT    // 选择云台类型
#define SHOOT_TYPE SHOOT_FRIC            // 选择发射机构类型
#define CONTROL_TYPE CHASSIS_AND_GIMBAL  // 选择控制类型

// 机器人物理参数
typedef enum {
    // 底盘CAN1
    WHEEL1 = 0,
    WHEEL2 = 1,
    WHEEL3 = 2,
    WHEEL4 = 3,
    // 云台CAN2
    YAW = 4,
    PITCH = 5,
    TRIGGER = 6,
    FRIC1 = 0,
    FRIC2 = 1,
} DJIMotorIndex_e;//DJI电机在接收数据数组中的索引

/*-------------------- Chassis --------------------*/
//physical parameters ---------------------
#define WHEEL_RADIUS 0.106f  //(m)轮子半径
#define WHEEL_DIRECTION 1    //轮子方向
//upper_limit parameters ---------------------
#define MAX_SPEED_VECTOR_VX 5.0f
#define MAX_SPEED_VECTOR_VY 5.0f
#define MAX_SPEED_VECTOR_WZ 1.0f

//lower_limit parameters ---------------------
#define MIN_SPEED_VECTOR_VX -MAX_SPEED_VECTOR_VX
#define MIN_SPEED_VECTOR_VY -MAX_SPEED_VECTOR_VY
#define MIN_SPEED_VECTOR_WZ -MAX_SPEED_VECTOR_WZ

//PID parameters ---------------------
//驱动轮速度环PID参数
#define KP_CHASSIS_WHEEL_SPEED 0.0f
#define KI_CHASSIS_WHEEL_SPEED 0.0f
#define KD_CHASSIS_WHEEL_SPEED 0.0f
#define MAX_IOUT_CHASSIS_WHEEL_SPEED 0.0f
#define MAX_OUT_CHASSIS_WHEEL_SPEED 0.0f

//云台跟随角度环PID参数
#define KP_CHASSIS_GIMBAL_FOLLOW_ANGLE 0.0f
#define KI_CHASSIS_GIMBAL_FOLLOW_ANGLE 0.0f
#define KD_CHASSIS_GIMBAL_FOLLOW_ANGLE 0.0f
#define MAX_IOUT_CHASSIS_GIMBAL_FOLLOW_ANGLE 0.0f
#define MAX_OUT_CHASSIS_GIMBAL_FOLLOW_ANGLE 0.0f

/*-------------------- Gimbal --------------------*/
//motor parameters ---------------------
//电机id
#define uint8_t GIMBAL_DIRECT_YAW_ID 0x1ff
#define uint8_t GIMBAL_DIRECT_PITCH_ID 5

//电机can口
#define uint8_t GIMBAL_DIRECT_YAW_CAN 1
#define uint8_t GIMBAL_DIRECT_PITCH_CAN 2 

//电机种类
#define MotorType_e GIMBAL_DIRECT_YAW_MOTOR_TYPE
#define MotorType_e GIMBAL_DIRECT_PITCH_MOTOR_TYPE

//旋转方向
#define int8_t GIMBAL_DIRECT_YAW_DIRECTION
#define int8_t GIMBAL_DIRECT_PITCH_DIRECTION

//减速比
#define float GIMBAL_DIRECT_YAW_REDUCTION_RATIO
#define float GIMBAL_DIRECT_PITCH_REDUCTION_RATIO

//电机运行模式
#define uint16_t GIMBAL_DIRECT_YAW_MODE
#define uint16_t GIMBAL_DIRECT_PITCH_MODE
//physical parameters ---------------------
#define GIMBAL_UPPER_LIMIT_PITCH 0.0f
#define GIMBAL_UPPER_LIMIT_YAW 0.0f
#define GIMBAL_LOWER_LIMIT_PITCH 0.0f
#define GIMBAL_LOWER_LIMIT_YAW 0.0f
//PID parameters ---------------------
//YAW ANGLE
#define KP_GIMBAL_YAW_ANGLE 0.0f
#define KI_GIMBAL_YAW_ANGLE 0.0f
#define KD_GIMBAL_YAW_ANGLE 0.0f
#define MAX_IOUT_GIMBAL_YAW_ANGLE 0.0f
#define MAX_OUT_GIMBAL_YAW_ANGLE 0.0f
//VELOCITY:角速度
#define KP_GIMBAL_YAW_VELOCITY 0.0f
#define KI_GIMBAL_YAW_VELOCITY 0.0f
#define KD_GIMBAL_YAW_VELOCITY 0.0f
#define MAX_IOUT_GIMBAL_YAW_VELOCITY 0.0f
#define MAX_OUT_GIMBAL_YAW_VELOCITY 0.0f

//PITCH ANGLE
#define KP_GIMBAL_PITCH_ANGLE 0.0f
#define KI_GIMBAL_PITCH_ANGLE 0.0f
#define KD_GIMBAL_PITCH_ANGLE 0.0f
#define MAX_IOUT_GIMBAL_PITCH_ANGLE 0.0f
#define MAX_OUT_GIMBAL_PITCH_ANGLE 0.0f
//VELOCITY:角速度
#define KP_GIMBAL_PITCH_VELOCITY 0.0f
#define KI_GIMBAL_PITCH_VELOCITY 0.0f
#define KD_GIMBAL_PITCH_VELOCITY 0.0f
#define MAX_IOUT_GIMBAL_PITCH_VELOCITY 0.0f
#define MAX_OUT_GIMBAL_PITCH_VELOCITY 0.0f
/*-------------------- Shoot --------------------*/
//physical parameters ---------------------
#define FRIC_RADIUS 0.03f  // (m)摩擦轮半径
#define BULLET_NUM 8       // 定义拨弹盘容纳弹丸个数
#define GUN_NUM 1          // 定义枪管个数（一个枪管2个摩擦轮）

//PID parameters ---------------------

#endif /* INCLUDED_ROBOT_PARAM_H */
