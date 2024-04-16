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
#include "gimbal_console.h"
#include "gimbal.h"

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
uint32_t gimbal_high_water;
#endif

static void InitGimbal(Gimbal_t *gimbal);

static void SetGimbalMode(Gimbal_t *gimbal);

static void GimbalObserver(Gimbal_t *gimbal);

static void GimbalReference(Gimbal_t *gimbal);

static void GimbalConsole(Gimbal_t *gimbal);

static void SendGimbalCmd(Gimbal_t *gimbal);

/**
 * @brief          云台任务，间隔 GIMBAL_CONTROL_TIME
 * @param[in]      pvParameters: 空
 * @retval         none
 */

void gimbal_task(void const *pvParameters)
{
    // 等待陀螺仪任务更新陀螺仪数据
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    // 云台初始化
    InitGimbal(&gimbal);

    while (1)
    {
        // 设置云台模式
        SetGimbalMode(&gimbal);
        // 更新状态量
        GimbalObserver(&gimbal);
        // 更新目标量
        GimbalReference(&gimbal);
        // 计算控制量
        GimbalConsole(&gimbal);
        // 发送控制量
        SendGimbalCmd(&gimbal);
        // 系统延时
        vTaskDelay(GIMBAL_CONTROL_TIME);
    }
}

/**
 * @brief          初始化
 * @param[in]      gimbal 云台结构体指针
 * @retval         none
 */
static void InitGimbal(Gimbal_t *gimbal)
{
}

/**
 * @brief          设置模式
 * @param[in]      gimbal 云台结构体指针
 * @retval         none
 */
static void SetGimbalMode(Gimbal_t *gimbal)
{
}

/**
 * @brief          更新状态量
 * @param[in]      gimbal 云台结构体指针
 * @retval         none
 */
static void GimbalObserver(Gimbal_t *gimbal)
{
}

/**
 * @brief          更新目标量
 * @param[in]      gimbal 云台结构体指针
 * @retval         none
 */
static void GimbalReference(Gimbal_t *gimbal)
{
}

/**
 * @brief          计算控制量
 * @param[in]      gimbal 云台结构体指针
 * @retval         none
 */
static void GimbalConsole(Gimbal_t *gimbal)
{
}

/**
 * @brief          发送控制量
 * @param[in]      gimbal 云台结构体指针
 * @retval         none
 */
static void SendGimbalCmd(Gimbal_t *gimbal)
{
}
