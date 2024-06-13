/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       custom_controller_task.c/h
  * @brief      完成机械臂控制任务
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     May-27-2024     Penguin         1. 完成基本框架
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */
#include "custom_controller_task.h"

#include "cmsis_os.h"
#include "custom_controller.h"

#ifndef __weak
#define __weak __attribute__((weak))
#endif /* __weak */

__weak void CustomControllerPublish(void);
__weak void CustomControllerInit(void);
__weak void CustomControllerHandleException(void);
__weak void CustomControllerSetMode(void);
__weak void CustomControllerObserver(void);
__weak void CustomControllerReference(void);
__weak void CustomControllerConsole(void);
__weak void CustomControllerSendCmd(void);

/**
 * @brief          自定义控制器任务
 * @param[in]      pvParameters: 空
 * @retval         none
 */
void custom_controller_task(void const * pvParameters)
{
    CustomControllerPublish();
    vTaskDelay(CUSTOM_CONTROLLER_TASK_INIT_TIME);
    // 初始化
    CustomControllerInit();

    while (1) {
        // 更新状态量
        CustomControllerObserver();
        // 处理异常
        CustomControllerHandleException();
        // 设置模式
        CustomControllerSetMode();
        // 设置目标量
        CustomControllerReference();
        // 计算控制量
        CustomControllerConsole();
        // 发送控制量
        CustomControllerSendCmd();

        // 系统延时
        vTaskDelay(CUSTOM_CONTROLLER_CONTROL_TIME);
    }
}

__weak void CustomControllerPublish(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void CustomControllerInit(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void CustomControllerHandleException(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void CustomControllerSetMode(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void CustomControllerObserver(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void CustomControllerReference(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void CustomControllerConsole(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void CustomControllerSendCmd(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
