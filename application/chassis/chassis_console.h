/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       chassis_console.c/h
  * @brief      底盘控制器。
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

#ifndef CHASSIS_CONSOLE_H
#define CHASSIS_CONSOLE_H

#include "chassis.h"

/*-------------------- Console --------------------*/
#if (CHASSIS_TYPE == CHASSIS_MECANUM_WHEEL)

#elif (CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)

#elif (CHASSIS_TYPE == CHASSIS_STEERING_WHEEL)

#elif (CHASSIS_TYPE == CHASSIS_BALANCE)
#include "leg_model.h"
void BalanceChassisConsole(Chassis_s *chassis);

#endif

#endif //CHASSIS_CONSOLE_H
