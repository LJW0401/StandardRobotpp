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
  *  V1.0.2     Jun-13-2024     Penguin         1. 添加默认的任务控制时间类宏定义
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */
#include "chassis_task.h"

#include "chassis.h"
#include "chassis_balance.h"
#include "chassis_mecanum.h"
#include "chassis_omni.h"
#include "chassis_steering.h"
#include "cmsis_os.h"

#ifndef CHASSIS_TASK_INIT_TIME
#define CHASSIS_TASK_INIT_TIME 357
#endif // CHASSIS_TASK_INIT_TIME

#ifndef CHASSIS_CONTROL_TIME_MS
#define CHASSIS_CONTROL_TIME_MS 2
#endif // CHASSIS_CONTROL_TIME_MS

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif

#ifndef __weak
#define __weak __attribute__((weak))
#endif /* __weak */

__weak void ChassisPublish(void);
__weak void ChassisInit(void);
__weak void ChassisHandleException(void);
__weak void ChassisSetMode(void);
__weak void ChassisObserver(void);
__weak void ChassisReference(void);
__weak void ChassisConsole(void);
__weak void ChassisSendCmd(void);

/**
 * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
 * @param[in]      pvParameters: 空
 * @retval         none
 */
void chassis_task(void const * pvParameters)
{
    ChassisPublish();
    // 空闲一段时间
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    // 初始化底盘
    ChassisInit();

    while (1) {
        // 更新状态量
        ChassisObserver();
        // 处理异常
        ChassisHandleException();
        // 设置底盘模式
        ChassisSetMode();
        // 更新目标量
        ChassisReference();
        // 计算控制量
        ChassisConsole();
        // 发送控制量
        ChassisSendCmd();
        // 系统延时
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

__weak void ChassisPublish(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void ChassisInit(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void ChassisHandleException(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void ChassisSetMode(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void ChassisObserver(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void ChassisReference(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void ChassisConsole(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void ChassisSendCmd(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
