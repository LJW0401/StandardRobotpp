/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       motor.c/h
  * @brief      电机相关部分定义
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024      Penguin         1. done
  *  V1.0.0     May-5-2024      Penguin         1. 添加dji电机的速度和位置控制
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#include "motor.h"

#include "cmsis_os.h"
#include "pid.h"

/**
 * @brief       电机初始化
 * @param[in]   p_motor 电机结构体
 * @param[in]   id 电机id
 * @param[in]   can 电机使用的can口
 * @param[in]   motor_type 电机种类
 * @param[in]   direction 电机旋转方向
 */
void MotorInit(Motor_s * p_motor, uint8_t id, uint8_t can, MotorType_e motor_type, int8_t direction)
{
    p_motor->id = id;
    p_motor->can = can;
    p_motor->type = motor_type;
    p_motor->direction = direction;
}

/**
 * @brief       dji电机速度控制
 * @param[in]   p_motor 
 * @param[in]   pid 
 * @param[in]   velocity 
 * @param[in]   feedforward 
 */
void DjiMotorVelocityControl(
    Motor_s * p_motor, pid_type_def * pid, float velocity, float feedforward)
{
    if (p_motor == NULL || pid == NULL) return;
    if (p_motor->type != DJI_M2006 && p_motor->type != DJI_M3508 && p_motor->type != DJI_M6020)
        return;
    p_motor->current_set = PID_calc(pid, p_motor->w, velocity) + feedforward;
}

/**
 * @brief       dji电机位置控制
 * @param[in]   p_motor 
 * @param[in]   pid 
 * @param[in]   velocity 
 * @param[in]   feedforward 
 */
void DjiMotorPositionControl(
    Motor_s * p_motor, pid_type_def * angle_pid, pid_type_def * velocity_pid, float angle,
    float feedforward)
{
    if (p_motor == NULL || angle_pid == NULL || velocity_pid == NULL) return;
    if (p_motor->type != DJI_M2006 && p_motor->type != DJI_M3508 && p_motor->type != DJI_M6020)
        return;
    float velocity_set = PID_calc(angle_pid, p_motor->pos, angle);
    float current_set = PID_calc(velocity_pid, p_motor->w, velocity_set) + feedforward;
}
