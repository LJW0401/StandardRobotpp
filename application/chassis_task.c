/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             底盘控制任务
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024     Penguin          1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */
#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "chassis_calculate.h"

#include "cmsis_os.h"

#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "INS_task.h"
#include "chassis_power_control.h"

#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif

/**
 * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
 * @param[in]      pvParameters: 空
 * @retval         none
 */
void chassis_task(void const *pvParameters)
{
    // 空闲一段时间
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    // 初始化底盘
    InitChassis();

    while (1)
    {
        // 设置底盘模式
        SetChassisMode();
        // 更新底盘数据（更新状态量）
        UpdateChassisData();
        // 底盘控制（计算控制量）
        ChassisConsole();
        // 发送底盘控制量
        SendChassisCmd();
        // 系统延时
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/**
 * @brief    底盘初始化
 * @param    none
 */
static void InitChassis(void)
{
}

/**
 * @brief 
 * @param  
 */
static void SetChassisMode(void)
{
}

/**
 * @brief 
 * @param  
 */
static void UpdateChassisData(void)
{
}

/**
 * @brief    底盘控制器
 * @param    none
 */
static void ChassisConsole(void)
{
}

/**
 * @brief 
 * @param  
 */
static void SendChassisCmd(void)
{
}

/**
 * @brief
 * @param  none
 * @return true-全部在线 false-不全在线
 */
// static bool DetectChassisMotor(void)
// {
//     return true;
// }
