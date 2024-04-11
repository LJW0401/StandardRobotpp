/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024      Penguin         1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "shoot.h"
#include "main.h"

#include "cmsis_os.h"

#include "bsp_laser.h"
#include "bsp_fric.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"

#include "CAN_communication.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"

// PID相关宏定义
#define FRIC_KP 10.0f
#define FRIC_KI 0.0f
#define FRIC_KD 0.0f
#define FRIC_MAX_IOUT 1.0f
#define FRIC_MAX_OUT 3000.0f

Shoot_s shoot = {
    .mode = LOAD_STOP,

    .shoot_frequency = 10,
    .dangle = 2 * PI / BULLET_NUM,

    .trigger_pid = {
        .mode = PID_POSITION,
        .Kp = 0.1,
        .Ki = 0,
        .Kd = 0,
        .max_iout = 10,
        .max_out = 1000,
    },
};

const fp32 fric_Kpid[3] = {FRIC_KP, FRIC_KI, FRIC_KD};

static void FricConsole(Shoot_s *shoot);

static void TriggerLoad_1_Bullet(Shoot_s *shoot);

static void TriggerLoadBurstFire(Shoot_s *shoot);

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

    uint8_t i;
    shoot->rc = get_remote_control_point();
    // 获取电机指针
    for (i = 0; i < 4; i++)
    {
        shoot->fric_motor[i].motor_measure = GetDjiMotorMeasurePoint(2, i);
    }
    shoot->trigger_motor.motor_measure = GetDjiMotorMeasurePoint(2, TRIGGER);

    // PID 初始化
    for (i = 0; i < 4; i++)
    {
        PID_init(&shoot->fric_pid[i], PID_POSITION, fric_Kpid, FRIC_MAX_IOUT, FRIC_MAX_OUT);
    }
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

    if (switch_is_up(shoot->rc->rc.s[SHOOT_MODE_CHANNEL]))
    {
        shoot->mode = LOAD_BURSTFIRE;
    }
    else if (switch_is_mid(shoot->rc->rc.s[SHOOT_MODE_CHANNEL]))
    {
        shoot->mode = LOAD_1_BULLET;
    }
    else if (switch_is_down(shoot->rc->rc.s[SHOOT_MODE_CHANNEL]))
    {
        shoot->mode = LOAD_STOP;
    }
}

/**
 * @brief          设置射击目标量
 * @param[in]      shoot 射击结构体指针
 */
void SetShootTarget(Shoot_s *shoot)
{
    switch (shoot->mode)
    {
    case LOAD_STOP:
        shoot->shoot_speed = 0;
        break;
    case LOAD_1_BULLET:
        shoot->shoot_speed = 15;
        break;
    case LOAD_BURSTFIRE:
        shoot->shoot_speed = 15;
        break;
    default:
        shoot->shoot_speed = 0;
        break;
    }
}

/**
 * @brief          更新射击数据（更新状态量）
 * @param[in]      shoot 射击结构体指针
 */
void UpdateShootData(Shoot_s *shoot)
{
    if (shoot == NULL)
    {
        return;
    }

    uint8_t i;
    for (i = 0; i < 4; i++)
    {
        shoot->fric_motor[i].w = shoot->fric_motor[i].motor_measure->speed_rpm * RPM_TO_OMEGA;
        shoot->fric_motor[i].v = shoot->fric_motor[i].w * FRIC_RADIUS;
    }

    shoot->trigger_motor.w = shoot->trigger_motor.motor_measure->speed_rpm * DJI_GM2006_RPM_TO_OMEGA;
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

    FricConsole(shoot);

    switch (shoot->mode)
    {
    case LOAD_STOP:
        break;
    case LOAD_1_BULLET:
        break;
    case LOAD_BURSTFIRE:
        break;
    default:
        break;
    }
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

    if (toe_is_error(DBUS_TOE))
    {
        // 发送摩擦轮电机控制电流
        CAN_CmdDJIMotor(&DJI_Motor_Send_Data_CAN2_0x200,
                        0, 0, 0, 0);
    }
    else
    {
        // 发送摩擦轮电机控制电流
        CAN_CmdDJIMotor(&DJI_Motor_Send_Data_CAN2_0x200,
                        shoot->fric_motor[0].current_set,
                        shoot->fric_motor[1].current_set,
                        shoot->fric_motor[2].current_set,
                        shoot->fric_motor[3].current_set);
    }
}

/**
 * @brief          摩擦轮控制
 * @param[in]      shoot 射击结构体指针
 */
static void FricConsole(Shoot_s *shoot)
{
    if (shoot == NULL)
    {
        return;
    }

    uint8_t i;
    float speed = shoot->shoot_speed;
    for (i = 0; i < 4; i++)
    {
        PID_calc(&shoot->fric_pid[i], shoot->fric_motor[i].v, speed);
        shoot->fric_motor[i].current_set = shoot->fric_pid[i].out;
        speed = -speed;
    }
}

/**
 * @brief          拨弹盘单发位控
 * @param[in]      shoot 射击结构体指针
 */
static void TriggerLoad_1_Bullet(Shoot_s *shoot)
{
    if (shoot == NULL)
    {
        return;
    }
}

/**
 * @brief          拨弹盘连发速控
 * @param[in]      shoot 射击结构体指针
 */
static void TriggerLoadBurstFire(Shoot_s *shoot)
{
    if (shoot == NULL)
    {
        return;
    }
    float omega = shoot->shoot_frequency / BULLET_NUM * 2 * PI;
    PID_calc(&shoot->trigger_pid, shoot->trigger_motor.w, omega);
    shoot->trigger_motor.current_set = shoot->trigger_pid.out;
}
