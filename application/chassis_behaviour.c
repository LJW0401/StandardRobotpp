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
#include "chassis_behaviour.h"

/*-------------------- Console --------------------*/
#if (CHASSIS_TYPE == CHASSIS_MECANUM_WHEEL)

#elif (CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)

#elif (CHASSIS_TYPE == CHASSIS_STEERING_WHEEL)

#elif (CHASSIS_TYPE == CHASSIS_BALANCE)
static void LocomotionController();
static void LegController();
/**
 * @brief 平衡底盘控制器
 * @param chassis
 */
void BalanceChassisConsole(Chassis_s *chassis)
{
    switch (chassis->mode)
    {
    case CHASSIS_ZERO_FORCE:
        break;
    case CHASSIS_FOLLOW_GIMBAL_YAW:
        double k_res[12];
        float k[2][6]; // LQR反馈矩阵
        L2K(0, k_res);
        float res[2];
        float T = res[0];  // 沿摆杆径向的力
        float Tp = res[1]; // 沿摆杆法向的力
        break;
    case CHASSIS_STOP:
        break;
    case CHASSIS_FREE:
        break;
    case CHASSIS_SPIN:
        break;
    case CHASSIS_AUTO:
        break;
    case CHASSIS_OPEN:
        break;
    default:
        break;
    }
}
/**
 * @brief 运动控制器
 */
static void LocomotionController() {}
/**
 * @brief 腿部控制器
 */
static void LegController() {}

#endif
