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

#include "chassis_calculate.h"

#if (CHASSIS_TYPE == CHASSIS_MECANUM_WHEEL)
/**
 * @brief          麦克纳姆轮速度解算
 * @param[in]      speed_vector_set: 设置的底盘速度向量
 * @param[out]     wheel_speed: 4个麦轮速度
 * @retval         none
 */
void ChassisSpeedVectorToMecanumWheelSpeed(const ChassisSpeedVector_t *speed_vector_set, fp32 wheel_speed[4])
{
    // TODO: add code here
    //  //because the gimbal is in front of chassis, when chassis rotates, wheel 0 and wheel 1 should be slower and wheel 2 and wheel 3 should be faster
    //  //旋转的时候， 由于云台靠前，所以是前面两轮 0 ，1 旋转的速度变慢， 后面两轮 2,3 旋转的速度变快
    //  wheel_speed[0] = -vx_set - vy_set + (WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    //  wheel_speed[1] = vx_set - vy_set + (WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    //  wheel_speed[2] = vx_set + vy_set + (-WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    //  wheel_speed[3] = -vx_set + vy_set + (-WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
}
#elif (CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)
/**
 * @brief          全向轮速度解算
 * @param[in]      speed_vector_set 设置的底盘速度向量
 * @param[out]     wheel_speed      4个全向轮速度
 */
void ChassisSpeedVectorToOmniWheelSpeed(const ChassisSpeedVector_t *speed_vector_set, fp32 wheel_speed[4])
{
    wheel_speed[0] = (-speed_vector_set->vx - speed_vector_set->vy) / 1.414f - speed_vector_set->wz * (LENGTH_L / 2);
    wheel_speed[1] = (speed_vector_set->vx - speed_vector_set->vy) / 1.414f - speed_vector_set->wz * (LENGTH_L / 2);
    wheel_speed[2] = (speed_vector_set->vx + speed_vector_set->vy) / 1.414f - speed_vector_set->wz * (LENGTH_L / 2);
    wheel_speed[3] = (-speed_vector_set->vx + speed_vector_set->vy) / 1.414f - speed_vector_set->wz * (LENGTH_L / 2);
}

/**
 * @brief          全向轮速度逆解算
 * @param[in]      wheel_speed  4个全向轮速度
 * @param[out]     speed_vector 底盘速度向量
 */
void OmniWheelSpeedToChassisSpeedVector(const fp32 wheel_speed[4], ChassisSpeedVector_t *speed_vector)
{
    speed_vector->vx = (wheel_speed[1] + wheel_speed[2] - wheel_speed[0] - wheel_speed[3]) / 4 * 1.414f;
    speed_vector->vy = (wheel_speed[2] + wheel_speed[3] - wheel_speed[0] - wheel_speed[1]) / 4 * 1.414f;
    speed_vector->wz = (wheel_speed[0] + wheel_speed[1] + wheel_speed[2] + wheel_speed[3]) / 4 / LENGTH_L;
}

#elif (CHASSIS_TYPE == CHASSIS_STEERING_WHEEL)
/**
 * @brief          舵轮速度解算
 * @param[in]      speed_vector_set: 设置的底盘速度向量
 * @param[out]     wheel_speed: 4个舵轮速度
 */
void ChassisSpeedVectorToSteeringWheelSpeed(const ChassisSpeedVector_t *speed_vector_set, fp32 wheel_speed[4])
{
    // TODO: add code here
}
#elif (CHASSIS_TYPE == CHASSIS_BALANCE)
/**
 * @brief          平衡速度解算
 * @param[in]      speed_vector_set: 设置的底盘速度向量
 * @param[out]     wheel_speed: 4个舵轮速度
 */
void ChassisSpeedVectorToSteeringWheelSpeed(const ChassisSpeedVector_t *speed_vector_set, fp32 wheel_speed[4])
{
    // TODO: add code here
}
#endif
