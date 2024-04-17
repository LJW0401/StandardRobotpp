/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       shoot.c/h
  * @brief      射击功能。
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

#ifndef SHOOT_H
#define SHOOT_H

// 任务相关宏定义
#define SHOOT_TASK_INIT_TIME 201  // 任务初始化 空闲一段时间
#define SHOOT_CONTROL_TIME 1      // 任务控制间隔 1ms

// 遥控器相关宏定义
#define SHOOT_MODE_CHANNEL 1  // 射击发射开关通道数据

typedef enum {
    LOAD_STOP,      // 停止拨盘
    LOAD_1_BULLET,  // 单发模式,根据鼠标按下的时间,触发一次之后需要进入不响应输入的状态(否则按下的时间内可能多次进入,导致多次发射)
    LOAD_BURSTFIRE  // 连发模式,对速度闭环
} LoadMode_e;

typedef enum {
    FRIC_NOT_READY = 0,  // 未准备发射
    FRIC_READY,          // 准备发射
} FricState_e;

#endif
