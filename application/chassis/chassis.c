/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       chassis.c/h
  * @brief      底盘控制任务所需要的变量和函数
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024      Penguin         1. done
  *  V1.0.1     Apr-16-2024     Penguin         1. 完成基本框架
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#include "chassis.h"

/*-------------------- Parameter definition --------------------*/
#if (CHASSIS_TYPE == CHASSIS_MECANUM_WHEEL)
#elif (CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)
#elif (CHASSIS_TYPE == CHASSIS_STEERING_WHEEL)
#elif (CHASSIS_TYPE == CHASSIS_BALANCE)
Chassis_s chassis = {
    .mode = CHASSIS_ZERO_FORCE,
    .yaw_mid = 0,
    .upper_limit = {
        .speed_vector = {
            .vx = 5.0f,
            .vy = 5.0f,
            .wz = 1.0f,
        },
        .roll = 1.0f,
        .yaw = M_PI,
        .x = {1.0f, 2.0f, 0.0f, 10.0f, 1.0f, 2.0f},
        .leg_length = 0.35f,
    },
    .lower_limit = {
        .speed_vector = {
            .vx = -5.0f,
            .vy = -5.0f,
            .wz = -1.0f,
        },
        .roll = -1.0f,
        .yaw = -M_PI,
        .x = {-1.0f, -2.0f, 0.0f, -10.0f, -1.0f, -2.0f},
        .leg_length = 0.15f,
    },
    .feedback = {
        .speed_vector = {
            .vx = 0.0f,
            .vy = 0.0f,
            .wz = 0.0f,
        },
        .roll = 0.0f,
        .yaw = 0.0f,
        .x = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        .leg_length = 0.0f,
    },
    .reference = {
        .speed_vector = {
            .vx = 0.0f,
            .vy = 0.0f,
            .wz = 0.0f,
        },
        .roll = 0.0f,
        .yaw = 0.0f,
        .x = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        .leg_length = 0.0f,
    },
    .dyaw = 0.0f,
};
#endif

/*-------------------- Init --------------------*/
#if (CHASSIS_TYPE == CHASSIS_MECANUM_WHEEL)
#elif (CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)
#elif (CHASSIS_TYPE == CHASSIS_STEERING_WHEEL)
#elif (CHASSIS_TYPE == CHASSIS_BALANCE)
/**
 * @brief 初始化底盘电机
 * @param[in] chassis
 */
void InitBalanceChassisMotor(Chassis_s *chassis)
{
    // chassis->joint_motor[0].motor = ;
    // chassis->joint_motor[1].motor = ;
    // chassis->joint_motor[2].motor = ;
    // chassis->joint_motor[3].motor = ;
    // chassis->wheel_motor[0].motor = ;
    // chassis->wheel_motor[1].motor = ;
}
#endif

/*-------------------- Observe --------------------*/
#if (CHASSIS_TYPE == CHASSIS_MECANUM_WHEEL)
#elif (CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)
#elif (CHASSIS_TYPE == CHASSIS_STEERING_WHEEL)
#elif (CHASSIS_TYPE == CHASSIS_BALANCE)
#endif

/*-------------------- Reference --------------------*/
#if (CHASSIS_TYPE == CHASSIS_MECANUM_WHEEL)
#elif (CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)
#elif (CHASSIS_TYPE == CHASSIS_STEERING_WHEEL)
#elif (CHASSIS_TYPE == CHASSIS_BALANCE)
#endif

/*-------------------- Cmd --------------------*/
#if (CHASSIS_TYPE == CHASSIS_MECANUM_WHEEL)
#elif (CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)
#elif (CHASSIS_TYPE == CHASSIS_STEERING_WHEEL)
#elif (CHASSIS_TYPE == CHASSIS_BALANCE)
static void SendJointMotorCmd(Chassis_s *chassis);
static void SendWheelMotorCmd(Chassis_s *chassis);
/**
 * @brief 发送平衡底盘控制指令
 * @param chassis
 */
void SendBalanceChassisCmd(Chassis_s *chassis)
{
    SendJointMotorCmd(chassis);
    SendWheelMotorCmd(chassis);
}
/**
 * @brief 发送关节电机控制指令
 * @param[in] chassis
 */
static void SendJointMotorCmd(Chassis_s *chassis)
{
}
/**
 * @brief 发送驱动轮电机控制指令
 * @param chassis
 */
static void SendWheelMotorCmd(Chassis_s *chassis)
{
}

#endif
