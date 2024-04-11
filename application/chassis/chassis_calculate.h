/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       chassis_calculate.c/h
  * @brief      这里是底盘解算部分
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-31-2024     Penguin         1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#ifndef CHASSIS_CALCULATE_H
#define CHASSIS_CALCULATE_H

#include "struct_typedef.h"
#include "robot_param.h"
#include "chassis.h"

#if (CHASSIS_TYPE == CHASSIS_MECANUM_WHEEL)
void ChassisSpeedVectorToMecanumWheelSpeed(const ChassisSpeedVector_t *speed_vector_set, fp32 wheel_speed[4]);
#elif (CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)
void ChassisSpeedVectorToOmniWheelSpeed(const ChassisSpeedVector_t *speed_vector_set, fp32 wheel_speed[4]);
void OmniWheelSpeedToChassisSpeedVector(const fp32 wheel_speed[4], ChassisSpeedVector_t *speed_vector);
#elif (CHASSIS_TYPE == CHASSIS_STEERING_WHEEL)
void ChassisSpeedVectorToSteeringWheelSpeed(const ChassisSpeedVector_t *speed_vector_set, fp32 wheel_speed[4]);
#elif (CHASSIS_TYPE == CHASSIS_BALANCE)
void ChassisSpeedVectorToSteeringWheelSpeed(const ChassisSpeedVector_t *speed_vector_set, fp32 wheel_speed[4]);
#endif

void GimbalSpeedVectorToChassisSpeedVector(ChassisSpeedVector_t *speed_vector_set, float dyaw);

#endif // CHASSIS_CALCULATE_H
