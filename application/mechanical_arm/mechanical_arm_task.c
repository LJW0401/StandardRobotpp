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
#include "mechanical_arm_5_axis.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t mechanical_arm_high_water;
#endif

/**
 * @brief          射击任务
 * @param[in]      pvParameters: 空
 * @retval         none
 */
void mechanical_arm_task(void const * pvParameters)
{
    vTaskDelay(MECHANICAL_ARM_TASK_INIT_TIME);
    // 射击初始化
    InitMechanicalArm();

    while (1) {
        // 设置模式
        SetMechanicalArmMode();
        // 更新状态量
        MechanicalArmObserver();
        // 设置目标量
        MechanicalArmReference();
        // 计算控制量
        MechanicalArmConsole();
        // 发送控制量
        SendMechanicalArmCmd();

        // 系统延时
        vTaskDelay(MECHANICAL_ARM_CONTROL_TIME);
    }
}
