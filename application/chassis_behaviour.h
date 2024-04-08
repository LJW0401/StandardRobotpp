/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       chassis_behaviour.c/h
  * @brief      根据遥控器的值，决定底盘行为。
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

#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H

#include "chassis.h"

/*-------------------- Console --------------------*/
#if (CHASSIS_TYPE == CHASSIS_MECANUM_WHEEL)

#elif (CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)

#elif (CHASSIS_TYPE == CHASSIS_STEERING_WHEEL)

#elif (CHASSIS_TYPE == CHASSIS_BALANCE)
#include "leg_model.h"
void BalanceChassisConsole(Chassis_s *chassis);

#endif

#endif
