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

#elif (CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)

#elif (CHASSIS_TYPE == CHASSIS_STEERING_WHEEL)

#elif (CHASSIS_TYPE == CHASSIS_BALANCE)
#include "leg_model.h"
void LocomotionController();
void LegController();

void InitBalanceChassisMotor(Chassis_s *chassis)
{
    // chassis->joint_motor[0].motor = ;
    // chassis->joint_motor[1].motor = ;
    // chassis->joint_motor[2].motor = ;
    // chassis->joint_motor[3].motor = ;
    // chassis->wheel_motor[0].motor = ;
    // chassis->wheel_motor[1].motor = ;
}



void BalanceConsole(Chassis_s *chassis)
{
    float x[6];    // 状态变量
    LegFKine();    // 获取五连杆等效连杆长度
    float k[2][6]; // LQR反馈矩阵
    L2K();
    float res[2];
    float T = res[0];  // 沿摆杆径向的力
    float Tp = res[1]; // 沿摆杆法向的力
}

void SendBalanceChassisCmd(Chassis_s *chassis)
{
    SendJointMotorCmd(chassis);
    SendWheelMotorCmd(chassis);
}

/**
 * @brief 运动控制器
 */
void LocomotionController(){}
/**
 * @brief 腿部控制器
 */
void LegController(){}

#endif
