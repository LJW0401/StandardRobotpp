/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       gimbal.c/h
  * @brief      云台部分通用变量和函数的定义
  * @note       将通用内容放在gimbal.c中，避免gimbal_task.c和gimbal_behaviour.c的循环引用
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024      Penguin          1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#ifndef GIMBAL_H
#define GIMBAL_H

/**
 * @brief 云台模式
 */
typedef enum
{
    GIMBAL_ZERO_FORCE, // 云台无力，所有控制量置0
    GIMBAL_GYRO,       // 云台陀螺仪控制（超出限位后转为编码角控制）
    GIMBAL_OPEN,       // 遥控器的值乘以比例成电流值开环控制
} GimbalMode_e;

#endif // GIMBAL_H