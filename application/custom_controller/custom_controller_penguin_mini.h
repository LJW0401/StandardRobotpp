/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       custom_controller_penguin_mini.c/h
  * @brief      企鹅mini自定义控制器。
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     May-27-2024     Penguin         1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/
#ifndef CUSTOM_CONTROLLER_PENGUIN_MINI_H
#define CUSTOM_CONTROLLER_PENGUIN_MINI_H

#include "robot_param.h"

#if (CUSTOM_CONTROLLER_TYPE == CUSTOM_CONTROLLER_PENGUIN_MINI)
#include "IMU_task.h"
#include "custom_controller.h"
#include "remote_control.h"

typedef struct
{
    const RC_ctrl_t * rc;  // 底盘使用的遥控器指针
    const Imu_t * imu;     // imu数据

    struct ctrl_data
    {
        uint16_t x;  // x方向
        uint16_t y;  // y方向
        uint16_t z;  // z方向
    } ctrl_data;     // 目标量

} CustomController_s;

extern void CustomControllerInit(void);

extern void CustomControllerHandleException(void);

extern void CustomControllerSetMode(void);

extern void CustomControllerObserver(void);

extern void CustomControllerReference(void);

extern void CustomControllerConsole(void);

extern void CustomControllerSendCmd(void);

#endif
#endif
