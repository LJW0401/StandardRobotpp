/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       motor.h
  * @brief      电机相关部分定义
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
#ifndef MOTOR_H
#define MOTOR_H

#include "struct_typedef.h"

#define DJI_GM6020_ECD_TO_RAD 0.000766990394f  // (2*pi/8192) 电机编码器值转换为弧度
#define DJI_GM3508_RPM_TO_OMEGA 0.0055115661f // (1/60*2*pi/19) m3508(减速比1:19) (rpm)->(rad/s)

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} DJI_Motor_Measure_t;

/**
 * @brief  DJI电机结构体
 * @note   包括电机的状态量和控制量
 */
typedef struct
{
    /*状态量*/
    const DJI_Motor_Measure_t *motor_measure; // 电机测量数据指针

    float accel;    //(rad/s^2)电机加速度
    float w;        //(rad/s)电机转速
    float v;        //(m/s)电机转速
    float position; //(rad)电机位置

    /*控制量*/
    float current_set; // 电机发送电流
} DJI_Motor_s;

#endif // MOTOR_H