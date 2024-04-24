/**
  * @file       robot_param_omni_infantry.h
  * @brief      这里是工程机器人参数配置文件，包括物理参数、PID参数等
  */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H
#include "robot_typedef.h"

#define CHASSIS_TYPE CHASSIS_NONE                  // 选择底盘类型
#define GIMBAL_TYPE GIMBAL_NONE                    // 选择云台类型
#define SHOOT_TYPE SHOOT_NONE                      // 选择发射机构类型
#define MECHANICAL_ARM_TYPE MECHANICAL_ARM_5_AXIS  // 选择机械臂类型
#define CONTROL_TYPE CHASSIS_AND_GIMBAL            // 选择控制类型

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
} DJIMotorIndex_e;  //DJI电机在接收数据数组中的索引

/*-------------------- Chassis --------------------*/
//motor type
#define WHEEL_MOTOR_TYPE DJI_M3508  //底盘驱动轮电机类型
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

/*-------------------- Mechanical arm --------------------*/
//motor type ---------------------
#define JOINT_MOTOR_0_TYPE CYBERGEAR_MOTOR
#define JOINT_MOTOR_1_TYPE CYBERGEAR_MOTOR
#define JOINT_MOTOR_2_TYPE CYBERGEAR_MOTOR
#define JOINT_MOTOR_3_TYPE DJI_M6020
#define JOINT_MOTOR_4_TYPE DJI_M3508

#define JOINT_MOTOR_0_ID 1
#define JOINT_MOTOR_1_ID 2
#define JOINT_MOTOR_2_ID 3
#define JOINT_MOTOR_3_ID 5
#define JOINT_MOTOR_4_ID 2

#define JOINT_MOTOR_0_CAN 1
#define JOINT_MOTOR_1_CAN 1
#define JOINT_MOTOR_2_CAN 1
#define JOINT_MOTOR_3_CAN 2
#define JOINT_MOTOR_4_CAN 2

#define JOINT_MOTOR_0_DIRECTION 1
#define JOINT_MOTOR_1_DIRECTION 1
#define JOINT_MOTOR_2_DIRECTION 1
#define JOINT_MOTOR_3_DIRECTION 1
#define JOINT_MOTOR_4_DIRECTION 1
//upper_limit parameters ---------------------
#define MAX_JOINT_0_POSITION 6.283185f  //2*M_PI
#define MAX_JOINT_1_POSITION M_PI
#define MAX_JOINT_2_POSITION M_PI
#define MAX_JOINT_3_POSITION 6.283185f
#define MAX_JOINT_4_POSITION M_PI
//lower_limit parameters ---------------------
#define MIN_JOINT_0_POSITION 0.0f
#define MIN_JOINT_1_POSITION 0.0f
#define MIN_JOINT_2_POSITION 0.0f
#define MIN_JOINT_3_POSITION 0.0f
#define MIN_JOINT_4_POSITION 0.0f
//PID parameters ---------------------
//J3角度环PID参数
#define KP_JOINT_3_ANGLE 0.0f
#define KI_JOINT_3_ANGLE 0.0f
#define KD_JOINT_3_ANGLE 0.0f
#define MAX_IOUT_JOINT_3_ANGLE 0.0f
#define MAX_OUT_JOINT_3_ANGLE 0.0f
//J3速度环PID参数
#define KP_JOINT_3_SPEED 0.0f
#define KI_JOINT_3_SPEED 0.0f
#define KD_JOINT_3_SPEED 0.0f
#define MAX_IOUT_JOINT_3_SPEED 0.0f
#define MAX_OUT_JOINT_3_SPEED 0.0f
//J4角度环PID参数
#define KP_JOINT_4_ANGLE 0.0f
#define KI_JOINT_4_ANGLE 0.0f
#define KD_JOINT_4_ANGLE 0.0f
#define MAX_IOUT_JOINT_4_ANGLE 0.0f
#define MAX_OUT_JOINT_4_ANGLE 0.0f
//J4速度环PID参数
#define KP_JOINT_4_SPEED 0.0f
#define KI_JOINT_4_SPEED 0.0f
#define KD_JOINT_4_SPEED 0.0f
#define MAX_IOUT_JOINT_4_SPEED 0.0f
#define MAX_OUT_JOINT_4_SPEED 0.0f

#endif /* INCLUDED_ROBOT_PARAM_H */
