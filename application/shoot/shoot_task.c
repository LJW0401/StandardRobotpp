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

static void InitShoot(Shoot_s *shoot);

static void SetShootMode(Shoot_s *shoot);

static void ShootReference(Shoot_s *shoot);

static void ShootObserver(Shoot_s *shoot);

static void ShootConsole(Shoot_s *shoot);

static void SendShootCmd(Shoot_s *shoot);

/**
 * @brief          射击任务
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
        // 更新状态量
        ShootObserver(&shoot);
        // 设置目标量
        ShootReference(&shoot);
        // 计算控制量
        ShootConsole(&shoot);
        // 发送控制量
        SendShootCmd(&shoot);

        // 系统延时
        vTaskDelay(SHOOT_CONTROL_TIME);
    }
}

/**
 * @brief          射击初始化
 * @param[in]      shoot 射击结构体指针
 */
void InitShoot(Shoot_s *shoot)
{
    if (shoot == NULL)
    {
        return;
    }

    // uint8_t i;
    // shoot->rc = get_remote_control_point();
    // // 获取电机指针
    // for (i = 0; i < 4; i++)
    // {
    //     shoot->fric_motor[i].motor_measure = GetDjiMotorMeasurePoint(2, i);
    // }
    // shoot->trigger_motor.motor_measure = GetDjiMotorMeasurePoint(2, TRIGGER);

    // // PID 初始化
    // for (i = 0; i < 4; i++)
    // {
    //     PID_init(&shoot->fric_pid[i], PID_POSITION, fric_Kpid, FRIC_MAX_IOUT, FRIC_MAX_OUT);
    // }
}

/**
 * @brief          设置射击模式
 * @param[in]      shoot 射击结构体指针
 */
void SetShootMode(Shoot_s *shoot)
{
    if (shoot == NULL)
    {
        return;
    }

    // if (switch_is_up(shoot->rc->rc.s[SHOOT_MODE_CHANNEL]))
    // {
    //     shoot->mode = LOAD_BURSTFIRE;
    // }
    // else if (switch_is_mid(shoot->rc->rc.s[SHOOT_MODE_CHANNEL]))
    // {
    //     shoot->mode = LOAD_1_BULLET;
    // }
    // else if (switch_is_down(shoot->rc->rc.s[SHOOT_MODE_CHANNEL]))
    // {
    //     shoot->mode = LOAD_STOP;
    // }
}

/**
 * @brief          设置射击目标量
 * @param[in]      shoot 射击结构体指针
 */
void ShootReference(Shoot_s *shoot)
{
    // switch (shoot->mode)
    // {
    // case LOAD_STOP:
    //     shoot->shoot_speed = 0;
    //     break;
    // case LOAD_1_BULLET:
    //     shoot->shoot_speed = 15;
    //     break;
    // case LOAD_BURSTFIRE:
    //     shoot->shoot_speed = 15;
    //     break;
    // default:
    //     shoot->shoot_speed = 0;
    //     break;
    // }
}

/**
 * @brief          更新射击数据（更新状态量）
 * @param[in]      shoot 射击结构体指针
 */
void ShootObserver(Shoot_s *shoot)
{
    if (shoot == NULL)
    {
        return;
    }

    // uint8_t i;
    // for (i = 0; i < 4; i++)
    // {
    //     shoot->fric_motor[i].w = shoot->fric_motor[i].motor_measure->speed_rpm * RPM_TO_OMEGA;
    //     shoot->fric_motor[i].v = shoot->fric_motor[i].w * FRIC_RADIUS;
    // }

    // shoot->trigger_motor.w = shoot->trigger_motor.motor_measure->speed_rpm * DJI_GM2006_RPM_TO_OMEGA;
}

/**
 * @brief 射击控制（计算控制量）
 */
void ShootConsole(Shoot_s *shoot)
{
    if (shoot == NULL)
    {
        return;
    }

    // FricConsole(shoot);

    // switch (shoot->mode)
    // {
    // case LOAD_STOP:
    //     break;
    // case LOAD_1_BULLET:
    //     break;
    // case LOAD_BURSTFIRE:
    //     break;
    // default:
    //     break;
    // }
}

/**
 * @brief 发送射击控制量
 */
void SendShootCmd(Shoot_s *shoot)
{
    if (shoot == NULL)
    {
        return;
    }

    // if (toe_is_error(DBUS_TOE))
    // {
    //     // 发送摩擦轮电机控制电流
    //     CAN_CmdDJIMotor(&DJI_Motor_Send_Data_CAN2_0x200,
    //                     0, 0, 0, 0);
    // }
    // else
    // {
    //     // 发送摩擦轮电机控制电流
    //     CAN_CmdDJIMotor(&DJI_Motor_Send_Data_CAN2_0x200,
    //                     shoot->fric_motor[0].current_set,
    //                     shoot->fric_motor[1].current_set,
    //                     shoot->fric_motor[2].current_set,
    //                     shoot->fric_motor[3].current_set);
    // }
}

/**
 * @brief          摩擦轮控制
 * @param[in]      shoot 射击结构体指针
 */
//static void FricConsole(Shoot_s *shoot)
//{
//    if (shoot == NULL)
//    {
//        return;
//    }

//    // uint8_t i;
//    // float speed = shoot->shoot_speed;
//    // for (i = 0; i < 4; i++)
//    // {
//    //     PID_calc(&shoot->fric_pid[i], shoot->fric_motor[i].v, speed);
//    //     shoot->fric_motor[i].current_set = shoot->fric_pid[i].out;
//    //     speed = -speed;
//    // }
//}

/**
 * @brief          拨弹盘单发位控
 * @param[in]      shoot 射击结构体指针
 */
//static void TriggerLoad_1_Bullet(Shoot_s *shoot)
//{
//    if (shoot == NULL)
//    {
//        return;
//    }
//}

/**
 * @brief          拨弹盘连发速控
 * @param[in]      shoot 射击结构体指针
 */
//static void TriggerLoadBurstFire(Shoot_s *shoot)
//{
//    if (shoot == NULL)
//    {
//        return;
//    }
//    // float omega = shoot->shoot_frequency / BULLET_NUM * 2 * PI;
//    // PID_calc(&shoot->trigger_pid, shoot->trigger_motor.w, omega);
//    // shoot->trigger_motor.current_set = shoot->trigger_pid.out;
//}
