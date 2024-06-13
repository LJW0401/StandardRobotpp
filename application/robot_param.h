/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       robot_param.h
  * @brief      这里是机器人参数配置文件，包括底盘参数，物理参数等
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-31-2024     Penguin         1. done
  *  V1.0.1     Apr-16-2024     Penguin         1. 添加云台和发射机构类型
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#ifndef ROBOT_PARAM_H
#define ROBOT_PARAM_H

#include "robot_typedef.h"
#include "struct_typedef.h"

//导入具体的机器人参数配置文件
#include "robot_param_balanced_infantry.h"

// 选择机器人的各种类型
#define __DEVELOP 0                    // 开发模式
#define __DEBUG 0                      // 调试模式
#define __TUNING 0                     // 调参模式
#define __MUSIC_ON 0                   // 开启音乐
#define __TUNING_MODE TUNING_NONE      // 调参模式
#define __SELF_BOARD_ID 1  // 本板ID

#endif /* ROBOT_PARAM_H */
