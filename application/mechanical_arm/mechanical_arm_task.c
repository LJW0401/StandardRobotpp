/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       mechanical_arm_task.c/h
  * @brief      完成机械臂控制任务
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.1     Apr-21-2024     Penguin         1. 完成基本框架
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#include "mechanical_arm_task.h"

#include "cmsis_os.h"
#include "mechanical_arm.h"
#include "mechanical_arm_penguin_mini.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t mechanical_arm_high_water;
#endif

#ifndef __weak
#define __weak __attribute__((weak))
#endif /* __weak */

__weak void MechanicalArmInit(void);
__weak void MechanicalArmHandleException(void);
__weak void MechanicalArmSetMode(void);
__weak void MechanicalArmObserver(void);
__weak void MechanicalArmReference(void);
__weak void MechanicalArmConsole(void);
__weak void MechanicalArmSendCmd(void);

/**
 * @brief          射击任务
 * @param[in]      pvParameters: 空
 * @retval         none
 */
void mechanical_arm_task(void const * pvParameters)
{
    vTaskDelay(MECHANICAL_ARM_TASK_INIT_TIME);
    // 射击初始化
    MechanicalArmInit();

    while (1) {
        // 处理异常
        MechanicalArmHandleException();
        // 更新状态量
        MechanicalArmObserver();
        // 设置模式
        MechanicalArmSetMode();
        // 设置目标量
        MechanicalArmReference();
        // 计算控制量
        MechanicalArmConsole();
        // 发送控制量
        MechanicalArmSendCmd();

        // 系统延时
        vTaskDelay(MECHANICAL_ARM_CONTROL_TIME);
    }
}

__weak void MechanicalArmInit(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void MechanicalArmHandleException(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void MechanicalArmSetMode(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void MechanicalArmObserver(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void MechanicalArmReference(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void MechanicalArmConsole(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void MechanicalArmSendCmd(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
