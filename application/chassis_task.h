/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             底盘控制任务
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024     Penguin          1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include <stdbool.h>

#include "struct_typedef.h"
#include "CAN_receive.h"
#include "gimbal_task.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"

//in the beginning of task ,wait a time
//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357

//the channel num of controlling vertial speed 
//前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 1
//the channel num of controlling horizontal speed
//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 0

//in some mode, can use remote control to control rotation speed
//在特殊模式下，可以通过遥控器控制旋转
#define CHASSIS_WZ_CHANNEL 2

//the channel of choosing chassis mode,
//选择底盘状态 开关通道号
#define CHASSIS_MODE_CHANNEL 0
//rocker value (max 660) change to vertial speed (m/s) 
//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.006f
//rocker value (max 660) change to horizontal speed (m/s)
//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.005f
//in following yaw angle mode, rocker value add to angle 
//跟随底盘yaw模式下，遥控器的yaw遥杆（max 660）增加到车体角度的比例
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
//in not following yaw angle mode, rocker value change to rotation speed
//不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
#define CHASSIS_WZ_RC_SEN 0.01f

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

//rocker value deadline
//摇杆死区
#define CHASSIS_RC_DEADLINE 10

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f


#define MOTOR_DISTANCE_TO_CENTER 0.2f

//chassis task control time  2ms
//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//chassis task control time 0.002s
//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002f
//chassis control frequence, no use now.
//底盘任务控制频率，尚未使用这个宏
#define CHASSIS_CONTROL_FREQUENCE 500.0f
//chassis 3508 max motor control current
//底盘3508最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 16000.0f
//press the key, chassis will swing
//底盘摇摆按键
#define SWING_KEY KEY_PRESSED_OFFSET_CTRL
//chassi forward, back, left, right key
//底盘前后左右控制按键
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D

//m3508 rmp change to chassis speed,
//m3508转化成底盘速度(m/s)的比例，
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//single chassis motor max speed
//单个底盘电机最大速度
#define MAX_WHEEL_SPEED 4.0f
//chassis forward or back max speed
//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 2.0f
//chassis left or right max speed
//底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 1.5f

#define CHASSIS_WZ_SET_SCALE 0.1f

//when chassis is not set to move, swing max angle
//摇摆原地不动摇摆最大角度(rad)
#define SWING_NO_MOVE_ANGLE 0.7f
//when chassis is set to move, swing max angle
//摇摆过程底盘运动最大角度(rad)
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f

//chassis motor speed PID
//底盘电机速度环PID
#define M3505_MOTOR_SPEED_PID_KP 15000.0f
#define M3505_MOTOR_SPEED_PID_KI 10.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//chassis follow angle PID
//底盘旋转跟随PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 40.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 6.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.2f

static void InitChassis(void);

static void SetChassisMode(void);

static void UpdateChassisData(void);

static void ChassisConsole(void);

static void SendChassisCmd(void);

// static bool DetectChassisMotor(void);


/**
  * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
extern void chassis_task(void const *pvParameters);

#endif
