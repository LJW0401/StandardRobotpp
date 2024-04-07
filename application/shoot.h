/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024      Penguin         1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef SHOOT_H
#define SHOOT_H
#include "struct_typedef.h"
#include "robot_param.h"

#include "CAN_receive.h"
#include "remote_control.h"
#include "user_lib.h"
#include "pid.h"

// 遥控器相关宏定义
#define SHOOT_MODE_CHANNEL 1 // 射击发射开关通道数据

typedef enum
{
    LOAD_STOP,     // 停止拨盘
    LOAD_1_BULLET, // 单发模式,根据鼠标按下的时间,触发一次之后需要进入不响应输入的状态(否则按下的时间内可能多次进入,导致多次发射)
    LOAD_BURSTFIRE // 连发模式,对速度闭环
} LoadMode_e;

typedef enum
{
    FRIC_NOT_READY = 0, // 未准备发射
    FRIC_READY,         // 准备发射
} FricState_e;

typedef struct
{
    const RC_ctrl_t *rc; // 射击使用的遥控器指针
    LoadMode_e mode;     // 射击模式
    FricState_e state;   // 摩擦轮状态

    DJI_Motor_s fric_motor[4]; // 摩擦轮电机
    DJI_Motor_s trigger_motor; // 拨弹盘电机

    /*目标量*/
    float shoot_frequency; // (Hz)射频
    float shoot_speed;     // (m/s)射速
    float dangle;          // (rad)拨弹盘单次转动角度

    //pid
    pid_type_def trigger_pid;
    pid_type_def fric_pid[4];
} Shoot_s;

extern Shoot_s shoot;

void InitShoot(Shoot_s *shoot);

void SetShootMode(Shoot_s *shoot);

void SetShootTarget(Shoot_s *shoot);

void UpdateShootData(Shoot_s *shoot);

void ShootConsole(Shoot_s *shoot);

void SendShootCmd(Shoot_s *shoot);

#endif
