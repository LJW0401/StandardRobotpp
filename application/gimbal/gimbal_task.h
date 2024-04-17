/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task
  *             完成云台控制任务
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024     Penguin          1. done
  *  V1.0.1     Apr-16-2024    Penguin          1. 完成基本框架
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

/**
 * @brief          云台任务，间隔 GIMBAL_CONTROL_TIME 1ms
 * @param[in]      pvParameters: 空
 * @retval         none
 */
extern void gimbal_task(void const *pvParameters);

#endif
