/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       shoot_task.c/h
  * @brief      完成射击控制任务
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024      Penguin         1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#include "shoot_task.h"
#include "shoot.h"

#include "main.h"

#include "cmsis_os.h"

#include "detect_task.h"
#include "remote_control.h"
#include "IMU_task.h"
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
uint32_t shoot_high_water;
#endif

/**
 * @brief          云台任务，间隔 GIMBAL_CONTROL_TIME 1ms
 * @param[in]      pvParameters: 空
 * @retval         none
 */

void shoot_task(void const *pvParameters)
{
    // 等待陀螺仪任务更新陀螺仪数据
    vTaskDelay(SHOOT_TASK_INIT_TIME);
    // 射击初始化
    InitShoot(&shoot);

    while (1)
    {
        // 设置射击模式
        SetShootMode(&shoot);
        // 更新射击数据（更新状态量）
        UpdateShootData(&shoot);
        // 设置射击目标量
        SetShootTarget(&shoot);
        // 射击控制（计算控制量）
        ShootConsole(&shoot);
        // 发送射击控制量
        SendShootCmd(&shoot);

        // 系统延时
        vTaskDelay(SHOOT_CONTROL_TIME);
    }
}
