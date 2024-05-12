/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       shoot_task.c/h
  * @brief      完成射击控制任务
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

#include "shoot_task.h"

#include "cmsis_os.h"
#include "shoot.h"
#include "shoot_fric.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t shoot_high_water;
#endif

#ifndef __weak
#define __weak __attribute__((weak))
#endif /* __weak */

__weak void InitShoot(void);
__weak void SetShootMode(void);
__weak void ShootObserver(void);
__weak void ShootReference(void);
__weak void ShootConsole(void);
__weak void SendShootCmd(void);

/**
 * @brief          射击任务
 * @param[in]      pvParameters: 空
 * @retval         none
 */
void shoot_task(void const * pvParameters)
{
    // 等待陀螺仪任务更新陀螺仪数据
    vTaskDelay(SHOOT_TASK_INIT_TIME);
    // 射击初始化
    InitShoot();

    while (1) {
        // 设置射击模式
        SetShootMode();
        // 更新状态量
        ShootObserver();
        // 设置目标量
        ShootReference();
        // 计算控制量
        ShootConsole();
        // 发送控制量
        SendShootCmd();

        // 系统延时
        vTaskDelay(SHOOT_CONTROL_TIME);
    }
}

__weak void InitShoot(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void SetShootMode(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void ShootObserver(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void ShootReference(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void ShootConsole(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void SendShootCmd(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
