/**
****************************(C) COPYRIGHT 2019 DJI****************************
* @file       chassis_behaviour.c/h
* @brief      according to remote control, change the chassis behaviour.
*             根据遥控器的值，决定底盘行为。
* @note
* @history
*  Version    Date            Author          Modification
*  V1.0.0     Dec-26-2018     RM              1. done
*  V1.1.0     Nov-11-2019     RM              1. add some annotation
*
@verbatim
==============================================================================

==============================================================================
@endverbatim
****************************(C) COPYRIGHT 2019 DJI****************************
*/
#include "chassis_behaviour.h"

#if (CHASSIS_TYPE == CHASSIS_MECANUM_WHEEL)
void Mecanumxxx(void)
{
}

#elif (CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)
void Omnixxx(void)
{
}

#elif (CHASSIS_TYPE == CHASSIS_STEERING_WHEEL)
void Steeringxxx()
{
}

#elif (CHASSIS_TYPE == CHASSIS_BALANCE)
void BalanceConsole(Chassis_s *chassis)
{

}

#endif