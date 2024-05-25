/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       gimbal.c/h
  * @brief      云台控制任务所需要的变量和函数
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

#ifndef GIMBAL_H
#define GIMBAL_H

// 云台任务相关宏定义
#define GIMBAL_TASK_INIT_TIME 201  // 任务初始化 空闲一段时间
#define GIMBAL_CONTROL_TIME 1      // 云台任务控制间隔 1ms

// 云台的遥控器相关宏定义
#define GIMBAL_YAW_CHANNEL 2    // yaw控制通道
#define GIMBAL_PITCH_CHANNEL 3  // pitch控制通道
#define GIMBAL_MODE_CHANNEL 0   // 状态开关通道
#define GIMBAL_RC_DEADBAND 10   // 摇杆死区


#endif  // GIMBAL_H
