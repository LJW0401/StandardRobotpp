/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       motor.c/h
  * @brief      电机相关部分定义
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

#include "motor.h"

/**
 * @brief       电机初始化
 * @param[in]   p_motor 电机结构体
 * @param[in]   id 电机id
 * @param[in]   can 电机使用的can口
 * @param[in]   motor_type 电机种类
 */
void MotorInit(Motor_s * p_motor, uint8_t id, uint8_t can, MotorType_e motor_type)
{
    p_motor->id = id;
    p_motor->can = can;
    p_motor->type = motor_type;
}
