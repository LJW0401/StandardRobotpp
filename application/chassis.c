/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       chassis.c/h
  * @brief      底盘部分通用变量和函数的定义
  * @note       将通用内容放在chassis.c中，避免chassis_task.c和chassis_behaviour.c的循环引用
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024      Penguin          1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#include "chassis.h"


Chassis_s chassis;

#if (CHASSIS_TYPE == CHASSIS_MECANUM_WHEEL)
void InitMecanumChassisMotor(Chassis_s *chassis) {}
#elif (CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)
void InitOmniChassisMotor(Chassis_s *chassis)
{
  uint8_t i;
  for (i = 0; i < 4; i++)
  {
    chassis->motor[i].motor_measure = GetDjiMotorMeasurePoint(1, i);
  }
  chassis->yaw_motor->motor_measure = GetDjiMotorMeasurePoint(2, YAW);
}

#elif (CHASSIS_TYPE == CHASSIS_STEERING_WHEEL)
void InitSteeringChassisMotor(Chassis_s *chassis) {}

#elif (CHASSIS_TYPE == CHASSIS_BALANCE)
void InitBalanceChassisMotor(Chassis_s *chassis) {}
void SendBalanceChassisCmd(Chassis_s *chassis){}

#endif