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

#include "gimbal_task.h"

#include "cmsis_os.h"
#include "gimbal.h"
#include "gimbal_yaw_pitch.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_high_water;
#endif

/**
 * @brief          云台任务，间隔 GIMBAL_CONTROL_TIME
 * @param[in]      pvParameters: 空
 * @retval         none
 */

void gimbal_task(void const * pvParameters)
{
    // 等待陀螺仪任务更新陀螺仪数据
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    // 云台初始化
    InitGimbal();

    while (1) {
        // 设置云台模式
        SetGimbalMode();
        // 更新状态量
        GimbalObserver();
        // 更新目标量
        GimbalReference();
        // 计算控制量
        GimbalConsole();
        // 发送控制量
        SendGimbalCmd();
        // 系统延时
        vTaskDelay(GIMBAL_CONTROL_TIME);
    }
}
