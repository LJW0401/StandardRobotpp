/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       mechanical_arm.c/h
  * @brief      机械臂功能。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.1     Apr-21-2024     Penguin         1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#ifndef MECHANICAL_ARM_H
#define MECHANICAL_ARM_H

// 任务相关宏定义
#define MECHANICAL_ARM_TASK_INIT_TIME 201  // 任务初始化 空闲一段时间
#define MECHANICAL_ARM_CONTROL_TIME 1      // 任务控制间隔 1ms

typedef enum {
    MECHANICAL_ARM_ZERO_FORCE,
    MECHANICAL_ARM_INIT,
    MECHANICAL_ARM_FOLLOW,
} MechanicalArmMode_e;

#endif  // MECHANICAL_ARM_H
