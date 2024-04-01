/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task
  *             完成云台控制任务
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

#include "gimbal_task.h"
#include "gimbal_behaviour.h"

#include "main.h"

#include "cmsis_os.h"

#include "detect_task.h"
#include "remote_control.h"
#include "INS_task.h"
#include "shoot.h"

// motor enconde value format, range[0-8191]
// 电机编码值规整 0—8191
#define ecd_format(ecd)         \
    {                           \
        if ((ecd) > ECD_RANGE)  \
            (ecd) -= ECD_RANGE; \
        else if ((ecd) < 0)     \
            (ecd) += ECD_RANGE; \
    }

#define gimbal_total_pid_clear(gimbal_clear)                                                   \
    {                                                                                          \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid);   \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_relative_angle_pid);   \
        PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_gyro_pid);                    \
                                                                                               \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid); \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_relative_angle_pid); \
        PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_gyro_pid);                  \
    }

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_high_water;
#endif

static void InitGimbal(void);

static void SetGimbalMode(void);

static void SetGimbalTarget(void);

static void UpdateGimbalData(void);

static void GimbalConsole(void);

static void SendGimbalCmd(void);

/**
 * @brief          云台任务，间隔 GIMBAL_CONTROL_TIME 1ms
 * @param[in]      pvParameters: 空
 * @retval         none
 */

void gimbal_task(void const *pvParameters)
{
    // 等待陀螺仪任务更新陀螺仪数据
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    // 云台初始化
    InitGimbal();

    while (1)
    {
        // 设置云台模式
        SetGimbalMode();
        // 设置目标量
        SetGimbalTarget();
        // 更新云台数据（更新状态量）
        UpdateGimbalData();
        // 云台控制（计算控制量）
        GimbalConsole();
        // 发送云台控制量
        SendGimbalCmd();

        // 设置射击模式
        SetShootMode();
        // 设置射击目标量
        SetShootTarget();
        // 更新射击数据（更新状态量）
        UpdateShootData();
        // 射击控制（计算控制量）
        ShootConsole();
        // 发送射击控制量
        SendShootCmd();

        // 系统延时
        vTaskDelay(GIMBAL_CONTROL_TIME);
    }
}

/**
 * @brief
 * @param
 */
static void InitGimbal(void)
{
}

/**
 * @brief
 * @param
 */
static void SetGimbalMode(void)
{
}

/**
 * @brief 
 * @param  
 */
static void SetGimbalTarget(void)
{
}

/**
 * @brief
 * @param
 */
static void UpdateGimbalData(void)
{
}

/**
 * @brief
 * @param
 */
static void GimbalConsole(void)
{
}

/**
 * @brief
 * @param
 */
static void SendGimbalCmd(void)
{
}
