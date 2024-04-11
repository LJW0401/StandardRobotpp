/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task
  *             完成云台控制任务
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

#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"

// 云台任务相关宏定义
#define GIMBAL_TASK_INIT_TIME 201 // 任务初始化 空闲一段时间
#define GIMBAL_CONTROL_TIME 1     // 云台任务控制间隔 1ms

// 云台的遥控器相关宏定义
#define GIMBAL_YAW_CHANNEL 2   // yaw控制通道
#define GIMBAL_PITCH_CHANNEL 3 // pitch控制通道
#define GIMBAL_MODE_CHANNEL 0  // 状态开关通道
#define GIMBAL_RC_DEADBAND 10  // 摇杆死区

/**
 * @brief          云台任务，间隔 GIMBAL_CONTROL_TIME 1ms
 * @param[in]      pvParameters: 空
 * @retval         none
 */

extern void gimbal_task(void const *pvParameters);

#endif
