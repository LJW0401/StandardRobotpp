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

// 底盘任务相关宏定义
#define CHASSIS_TASK_INIT_TIME 357  // 任务开始空闲一段时间
#define CHASSIS_CONTROL_TIME_MS 2   // 底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME 0.002f // 底盘任务控制间隔 0.002s

// 底盘的遥控器相关宏定义
#define CHASSIS_MODE_CHANNEL 0 // 选择底盘状态 开关通道号
#define CHASSIS_X_CHANNEL 1    // 前后的遥控器通道号码
#define CHASSIS_Y_CHANNEL 0    // 左右的遥控器通道号码
#define CHASSIS_WZ_CHANNEL 4   // 旋转的遥控器通道号码
#define CHASSIS_RC_DEADLINE 10// 摇杆死区


/**
 * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
 * @param[in]      pvParameters: 空
 * @retval         none
 */
extern void chassis_task(void const *pvParameters);

#endif
