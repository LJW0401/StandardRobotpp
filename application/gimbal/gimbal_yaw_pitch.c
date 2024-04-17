/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       gimbal_yaw_pitch.c/h
  * @brief      yaw_pitch云台控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
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

#include "gimbal_yaw_pitch.h"
#if (GIMBAL_TYPE == GIMBAL_YAW_PITCH)
static Gimbal_t GIMBAL = {
  .mode = GIMBAL_ZERO_FORCE,

};

/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param[in]      none
 * @retval         none
 */
void InitGimbal(void) { GIMBAL.rc = get_remote_control_point(); }

/*-------------------- Set mode --------------------*/

/**
 * @brief          设置模式
 * @param[in]      none
 * @retval         none
 */
void SetGimbalMode(void)
{
    if (switch_is_up(GIMBAL.rc->rc.s[GIMBAL_MODE_CHANNEL])) {
        GIMBAL.mode = GIMBAL_ZERO_FORCE;
    } else if (switch_is_mid(GIMBAL.rc->rc.s[GIMBAL_MODE_CHANNEL])) {
        GIMBAL.mode = GIMBAL_GYRO;
    } else if (switch_is_down(GIMBAL.rc->rc.s[GIMBAL_MODE_CHANNEL])) {
        GIMBAL.mode = GIMBAL_OPEN;
    }
}

/*-------------------- Observe --------------------*/

/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void GimbalObserver(void) {}

/*-------------------- Reference --------------------*/

/**
 * @brief          更新目标量
 * @param[in]      none
 * @retval         none
 */
void GimbalReference(void) {}

/*-------------------- Console --------------------*/

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void GimbalConsole(void) {}

/*-------------------- Cmd --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */
void SendGimbalCmd(void) {}

#endif  // GIMBAL_YAW_PITCH
