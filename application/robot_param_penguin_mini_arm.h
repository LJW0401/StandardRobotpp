/**
  * @file       robot_param_penguin_mini_arm.h
  * @brief      这里是小企鹅的迷你机械臂的参数配置文件，包括物理参数、PID参数等
  */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H
#include "robot_typedef.h"

#define CHASSIS_TYPE CHASSIS_NONE                            // 选择底盘类型
#define GIMBAL_TYPE GIMBAL_NONE                              // 选择云台类型
#define SHOOT_TYPE SHOOT_NONE                                // 选择发射机构类型
#define MECHANICAL_ARM_TYPE MECHANICAL_ARM_PENGUIN_MINI_ARM  // 选择机械臂类型
#define CONTROL_TYPE CHASSIS_AND_GIMBAL                      // 选择控制类型

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

#define JOINT_MOTOR_0_REDUCTION_RATIO 1
#define JOINT_MOTOR_1_REDUCTION_RATIO 1
#define JOINT_MOTOR_2_REDUCTION_RATIO 1
#define JOINT_MOTOR_3_REDUCTION_RATIO 1
#define JOINT_MOTOR_4_REDUCTION_RATIO 1

#define JOINT_MOTOR_0_MODE 0
#define JOINT_MOTOR_1_MODE 0
#define JOINT_MOTOR_2_MODE 0
#define JOINT_MOTOR_3_MODE 0
#define JOINT_MOTOR_4_MODE 0
//upper_limit parameters ---------------------
#define MAX_JOINT_0_POSITION 1.07f
#define MAX_JOINT_1_POSITION 0.0f
#define MAX_JOINT_2_POSITION 2.2f
#define MAX_JOINT_3_POSITION 6.283185f
#define MAX_JOINT_4_POSITION M_PI
//lower_limit parameters ---------------------
#define MIN_JOINT_0_POSITION -MAX_JOINT_0_POSITION
#define MIN_JOINT_1_POSITION -3.75f
#define MIN_JOINT_2_POSITION -1.9f
#define MIN_JOINT_3_POSITION 0.0f
#define MIN_JOINT_4_POSITION 0.0f
//PID parameters ---------------------
//J3角度环PID参数
#define KP_JOINT_3_ANGLE 10.0f
#define KI_JOINT_3_ANGLE 1.0f
#define KD_JOINT_3_ANGLE 0.5f
#define MAX_IOUT_JOINT_3_ANGLE 1.0f
#define MAX_OUT_JOINT_3_ANGLE 10.0f
//J3速度环PID参数
#define KP_JOINT_3_SPEED 1000.0f
#define KI_JOINT_3_SPEED 15.0f
#define KD_JOINT_3_SPEED 0.0f
#define MAX_IOUT_JOINT_3_SPEED 3000.0f
#define MAX_OUT_JOINT_3_SPEED 15000.0f
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
// Init parameters ---------------------
#define JOINT_1_INIT_MAX_TORQUE 2.0f  //(N*m)关节1初始化时的最大力矩
#define JOINT_2_INIT_MAX_TORQUE 0.7f  //(N*m)关节2初始化时的最大力矩
#define JOINT_INIT_VELOCITY_SET 0.5f  //(rad/s)关节初始化时的转速
#define JOINT_1_INIT_TORQUE_SET 0.5f  //(N*m)
// Other parameters ---------------------
#define JOINT_MIN_VELOCITY 0.05f  //(rad/s)当关节速度低于此速度时认为关节已经停止运动了
#define JOINT_ZERO_THRESHOLD 0.01f     //(rad)关节置零时的阈值
#define JOINT_ZERO_THRESHOLD_RUN 0.2f  //(rad)关节运行时的阈值
#define J_1_J_2_DELTA_MAX 2.5f         //(rad)关节1和关节2之间的最大差值
#define J_1_J_2_DELTA_MIN 1.15f        //(rad)关节1和关节2之间的最大差值
#define J_0_ANGLE_TRANSFORM 0.0f       //(rad)关节0的角度偏移
#define J_1_ANGLE_TRANSFORM 0.0f       //(rad)关节1的角度偏移
#define J_2_ANGLE_TRANSFORM 1.0f       //(rad)关节2的角度偏移
#define J_3_ANGLE_TRANSFORM 3.0f       //(rad)关节3的角度偏移
#define J_4_ANGLE_TRANSFORM 0.0f       //(rad)关节4的角度偏移

#define JOINT_0_MAX_TORQUE 4.0f  //(N*m)关节0最大输出力矩
#define JOINT_1_MAX_TORQUE 2.5f  //(N*m)关节1最大输出力矩
#define JOINT_2_MAX_TORQUE 1.3f  //(N*m)关节2最大输出力矩

#define JOINT_VELOCITY_SET 0.8f  //(rad/s)关节运行时的转速

#define POS_MUTATION_THRESHOLD 1.0f  //(rad)位置突变阈值

#endif /* INCLUDED_ROBOT_PARAM_H */
