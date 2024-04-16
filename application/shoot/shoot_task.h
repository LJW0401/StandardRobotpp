/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       shoot_task.c/h
  * @brief      完成射击控制任务
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024      Penguin         1. done
  *  V1.0.1     Apr-16-2024     Penguin         1. 完成基本框架
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"

// 任务相关宏定义
#define SHOOT_TASK_INIT_TIME 201 // 任务初始化 空闲一段时间
#define SHOOT_CONTROL_TIME 1     // 任务控制间隔 1ms

// 遥控器相关宏定义
#define SHOOT_MODE_CHANNEL 1 // 射击发射开关通道数据

extern void shoot_task(void const *pvParameters);

#endif
