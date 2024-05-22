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
 * @param[in]   reduction_ratio 电机减速比（如电机反馈已经处理完了可以不用注意）
 * @param[in]   mode 电机运行模式（通常关节电机需要用到）
 */
void MotorInit(
    Motor_s * p_motor, uint8_t id, uint8_t can, MotorType_e motor_type, int8_t direction,
    float reduction_ratio, uint16_t mode)
{
    p_motor->id = id;
    p_motor->can = can;
    p_motor->type = motor_type;
    p_motor->direction = direction;
    p_motor->reduction_ratio = reduction_ratio;
    p_motor->mode = mode;

    p_motor->offline = true;
}

