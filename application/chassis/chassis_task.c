/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       chassis_task.c/h
  * @brief      chassis control task,
  *             底盘控制任务
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
#include "chassis_task.h"

#include "chassis_balance.h"
#include "chassis_mecanum.h"
#include "chassis_omni.h"
#include "chassis_steering.h"
#include "cmsis_os.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif

/**
 * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
 * @param[in]      pvParameters: 空
 * @retval         none
 */
void chassis_task(void const * pvParameters)
{
    // 空闲一段时间
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    // 初始化底盘
    InitChassis();

    while (1) {
        // 设置底盘模式
        SetChassisMode();
        // 更新状态量
        ChassisObserver();
        // 更新目标量
        ChassisReference();
        // 计算控制量
        ChassisConsole();
        // 发送控制量
        SendChassisCmd();
        // 系统延时
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}
