/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       chassis_balance.c/h
  * @brief      平衡底盘控制器。
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
#include "chassis_omni.h"
#if (CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)
#include "CAN_communication.h"
#include "leg_model.h"

static Chassis_s CHASSIS = {
  .mode = CHASSIS_ZERO_FORCE,
  .yaw_mid = 0,
  .dyaw = 0.0f,
  .upper_limit =
    {
      .wheel_speed = {0.0f, 0.0f, 0.0f, 0.0f},
      .speed_vector.vx = MAX_SPEED_VECTOR_VX,
      .speed_vector.vy = MAX_SPEED_VECTOR_VY,
      .speed_vector.wz = MAX_SPEED_VECTOR_WZ,
    },
  .lower_limit =
    {
      .wheel_speed = {0.0f, 0.0f, 0.0f, 0.0f},
      .speed_vector.vx = MIN_SPEED_VECTOR_VX,
      .speed_vector.vy = MIN_SPEED_VECTOR_VY,
      .speed_vector.wz = MIN_SPEED_VECTOR_WZ,
    },
  .feedback =
    {
      .wheel_speed = {0.0f, 0.0f, 0.0f, 0.0f},
      .speed_vector.vx = 0.0f,
      .speed_vector.vy = 0.0f,
      .speed_vector.wz = 0.0f,
    },
  .reference =
    {
      .wheel_speed = {0.0f, 0.0f, 0.0f, 0.0f},
      .speed_vector.vx = 0.0f,
      .speed_vector.vy = 0.0f,
      .speed_vector.wz = 0.0f,
    },

  .pid =
    {
      .wheel_pid_speed[0] =
        {
          .Kp = KP_CHASSIS_WHEEL_SPEED,
          .Ki = KI_CHASSIS_WHEEL_SPEED,
          .Kd = KD_CHASSIS_WHEEL_SPEED,
          .max_iout = MAX_IOUT_CHASSIS_WHEEL_SPEED,
          .max_out = MAX_OUT_CHASSIS_WHEEL_SPEED,
        },
      .wheel_pid_speed[1] =
        {
          .Kp = KP_CHASSIS_WHEEL_SPEED,
          .Ki = KI_CHASSIS_WHEEL_SPEED,
          .Kd = KD_CHASSIS_WHEEL_SPEED,
          .max_iout = MAX_IOUT_CHASSIS_WHEEL_SPEED,
          .max_out = MAX_OUT_CHASSIS_WHEEL_SPEED,
        },
      .wheel_pid_speed[2] =
        {
          .Kp = KP_CHASSIS_WHEEL_SPEED,
          .Ki = KI_CHASSIS_WHEEL_SPEED,
          .Kd = KD_CHASSIS_WHEEL_SPEED,
          .max_iout = MAX_IOUT_CHASSIS_WHEEL_SPEED,
          .max_out = MAX_OUT_CHASSIS_WHEEL_SPEED,
        },
      .wheel_pid_speed[3] =
        {
          .Kp = KP_CHASSIS_WHEEL_SPEED,
          .Ki = KI_CHASSIS_WHEEL_SPEED,
          .Kd = KD_CHASSIS_WHEEL_SPEED,
          .max_iout = MAX_IOUT_CHASSIS_WHEEL_SPEED,
          .max_out = MAX_OUT_CHASSIS_WHEEL_SPEED,
        },
      .gimbal_follow_pid_angle =
        {
          .Kp = KP_CHASSIS_GIMBAL_FOLLOW_ANGLE,
          .Ki = KI_CHASSIS_GIMBAL_FOLLOW_ANGLE,
          .Kd = KD_CHASSIS_GIMBAL_FOLLOW_ANGLE,
          .max_iout = MAX_IOUT_CHASSIS_GIMBAL_FOLLOW_ANGLE,
          .max_out = MAX_OUT_CHASSIS_GIMBAL_FOLLOW_ANGLE,
        },
    },
};

/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param[in]      none
 * @retval         none
 */
void InitChassis(void)
{
    CHASSIS.rc = get_remote_control_point();  // 获取遥控器指针
    /*-------------------- 初始化底盘电机 --------------------*/
    uint8_t i;
    for (i = 0; i < 4; i++) {
      MotorInit(&CHASSIS.wheel_motor[i], i, 1, DJI_M3508);
    }
}

/*-------------------- Set mode --------------------*/

/**
 * @brief          设置模式
 * @param[in]      none
 * @retval         none
 */
void SetChassisMode(void)
{
    if (switch_is_up(CHASSIS.rc->rc.s[CHASSIS_MODE_CHANNEL])) {
        CHASSIS.mode = CHASSIS_OPEN;
    } else if (switch_is_mid(CHASSIS.rc->rc.s[CHASSIS_MODE_CHANNEL])) {
        CHASSIS.mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    } else if (switch_is_down(CHASSIS.rc->rc.s[CHASSIS_MODE_CHANNEL])) {
        CHASSIS.mode = CHASSIS_ZERO_FORCE;
    }
}

/*-------------------- Observe --------------------*/

/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void ChassisObserver(void) {}

/*-------------------- Reference --------------------*/

/**
 * @brief          更新目标量
 * @param[in]      none
 * @retval         none
 */
void ChassisReference(void) {}

/*-------------------- Console --------------------*/

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void ChassisConsole(void)
{
    switch (CHASSIS.mode) {
        case CHASSIS_ZERO_FORCE: {
            uint8_t i;
            for (i = 0; i < 4; i++) {
                CHASSIS.wheel_motor[0].current_set = 0.0f;
            }
            break;
        }
        case CHASSIS_FOLLOW_GIMBAL_YAW:
            break;
        case CHASSIS_STOP:
            break;
        case CHASSIS_FREE:
            break;
        case CHASSIS_SPIN:
            break;
        case CHASSIS_AUTO:
            break;
        case CHASSIS_OPEN: {
            uint8_t i;
            uint16_t current;
            current = CHASSIS.rc->rc.ch[CHASSIS_X_CHANNEL] * 3000 * RC_TO_ONE;
            for (i = 0; i < 4; i++) {
                CHASSIS.wheel_motor[0].current_set = current;
            }
            break;
        }
        default:
            break;
    }
}

/*-------------------- Cmd --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */
void SendChassisCmd(void)
{
    CAN_CmdDJIMotor(
      &DJI_Motor_Send_Data_CAN1_0x200,  //底盘电机控制电流通过CAN1发送，使用的标识符为0x200
      CHASSIS.wheel_motor[0].current_set, CHASSIS.wheel_motor[1].current_set,
      CHASSIS.wheel_motor[2].current_set, CHASSIS.wheel_motor[3].current_set);
}

#endif /* CHASSIS_BALANCE */
