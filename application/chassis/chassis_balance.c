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
#include "chassis_balance.h"
#if (CHASSIS_TYPE == CHASSIS_BALANCE)
#include "leg_model.h"

static Chassis_s CHASSIS = {
  .mode = CHASSIS_ZERO_FORCE,
  .yaw_mid = 0,
  .upper_limit =
    {
      .speed_vector =
        {
          .vx = 5.0f,
          .vy = 5.0f,
          .wz = 1.0f,
        },
      .roll = 1.0f,
      .yaw = M_PI,
      .x = {1.0f, 2.0f, 0.0f, 10.0f, 1.0f, 2.0f},
      .leg_length = 0.35f,
    },
  .lower_limit =
    {
      .speed_vector =
        {
          .vx = -5.0f,
          .vy = -5.0f,
          .wz = -1.0f,
        },
      .roll = -1.0f,
      .yaw = -M_PI,
      .x = {-1.0f, -2.0f, 0.0f, -10.0f, -1.0f, -2.0f},
      .leg_length = 0.15f,
    },
  .feedback =
    {
      .speed_vector =
        {
          .vx = 0.0f,
          .vy = 0.0f,
          .wz = 0.0f,
        },
      .roll = 0.0f,
      .yaw = 0.0f,
      .x = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
      .leg_length = 0.0f,
    },
  .reference =
    {
      .speed_vector =
        {
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
    //TODO:add code here
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
        CHASSIS.mode = CHASSIS_AUTO;
    } else if (switch_is_mid(CHASSIS.rc->rc.s[CHASSIS_MODE_CHANNEL])) {
        CHASSIS.mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    } else if (switch_is_down(CHASSIS.rc->rc.s[CHASSIS_MODE_CHANNEL])) {
        CHASSIS.mode = CHASSIS_FREE;
    }
}

/*-------------------- Observe --------------------*/

/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void ChassisObserver(void)
{
    // 更新底盘陀螺仪数据
    double left_leg_pos[2];
    double right_leg_pos[2];
    LegFKine(0, 0, left_leg_pos);   // 获取五连杆等效连杆长度
    LegFKine(0, 0, right_leg_pos);  // 获取五连杆等效连杆长度

    // LegPosUpdate();

    CHASSIS.feedback.x[0] = CHASSIS.imu->angle.pitch;
    CHASSIS.feedback.x[1] = CHASSIS.imu->velocity.y;
    CHASSIS.feedback.x[2] = 0;
    // chassis.feedback->x[3] = (left_wheel.speed + right_wheel.speed) / 2 * WHEEL_RADIUS;
    CHASSIS.feedback.x[4] =
      (CHASSIS.feedback.leg_pos[0].angle + CHASSIS.feedback.leg_pos[1].angle) / 2 - M_PI_2 -
      CHASSIS.imu->angle.pitch;
    CHASSIS.feedback.x[5] = 0;

    CHASSIS.dyaw =
      (CHASSIS.yaw_motor->motor_measure->ecd * DJI_GM6020_ECD_TO_RAD - CHASSIS.yaw_mid);
}

/*-------------------- Reference --------------------*/

/**
 * @brief          更新目标量
 * @param[in]      none
 * @retval         none
 */
void ChassisReference(void)
{
    int16_t rc_x = 0, rc_y = 0, rc_wz = 0;
    rc_deadband_limit(CHASSIS.rc->rc.ch[CHASSIS_X_CHANNEL], rc_x, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(CHASSIS.rc->rc.ch[CHASSIS_Y_CHANNEL], rc_y, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(CHASSIS.rc->rc.ch[CHASSIS_WZ_CHANNEL], rc_wz, CHASSIS_RC_DEADLINE);

    ChassisSpeedVector_t v_set = {0.0f, 0.0f, 0.0f};
    if (CHASSIS.mode == CHASSIS_FREE)  // 底盘自由模式下，控制量为底盘坐标系下的速度
    {
        v_set.vx = rc_x * RC_TO_ONE * CHASSIS.upper_limit.speed_vector.vx;
        v_set.vy = rc_y * RC_TO_ONE * CHASSIS.upper_limit.speed_vector.vy;
        v_set.wz = rc_wz * RC_TO_ONE * CHASSIS.upper_limit.speed_vector.wz;
    } else if (
      CHASSIS.mode ==
      CHASSIS_FOLLOW_GIMBAL_YAW)  // 云台跟随模式下，控制量为云台坐标系下的速度，需要进行坐标转换
    {
        v_set.vx = rc_x * RC_TO_ONE * CHASSIS.upper_limit.speed_vector.vx;
        v_set.vy = rc_y * RC_TO_ONE * CHASSIS.upper_limit.speed_vector.vy;
        v_set.wz = rc_wz * RC_TO_ONE * CHASSIS.upper_limit.speed_vector.wz;
        GimbalSpeedVectorToChassisSpeedVector(&v_set, CHASSIS.dyaw);
    } else if (
      CHASSIS.mode == CHASSIS_AUTO)  // 底盘自动模式，控制量为云台坐标系下的速度，需要进行坐标转换
    {
        // TODO: add code here
    }
    CHASSIS.reference.speed_vector.vx = v_set.vx;
    CHASSIS.reference.speed_vector.vy = v_set.vy;
    CHASSIS.reference.speed_vector.wz = v_set.wz;

    CHASSIS.reference.x[0] = 0;
    CHASSIS.reference.x[1] = 0;
    CHASSIS.reference.x[2] = 0;
    CHASSIS.reference.x[3] = CHASSIS.reference.speed_vector.vx;
    CHASSIS.reference.x[4] = 0;
    CHASSIS.reference.x[5] = 0;
}

/*-------------------- Console --------------------*/
//static void LocomotionController(void);
//static void LegController(void);

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void ChassisConsole(void)
{
    switch (CHASSIS.mode) {
        case CHASSIS_ZERO_FORCE:
            break;
        case CHASSIS_FOLLOW_GIMBAL_YAW:
            //double k_res[12];
            //float k[2][6]; // LQR反馈矩阵
            //L2K(0, k_res);
            //float res[2];
            //float T = res[0];  // 沿摆杆径向的力
            //float Tp = res[1]; // 沿摆杆法向的力
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
//static void LocomotionController(void) {}
/**
 * @brief 腿部控制器
 */
//static void LegController(void) {}

/*-------------------- Cmd --------------------*/
static void SendJointMotorCmd(void);
static void SendWheelMotorCmd(void);

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */
void SendChassisCmd(void)
{
    SendJointMotorCmd();
    SendWheelMotorCmd();
}
/**
 * @brief 发送关节电机控制指令
 * @param[in] chassis
 */
static void SendJointMotorCmd(void) {}
/**
 * @brief 发送驱动轮电机控制指令
 * @param chassis
 */
static void SendWheelMotorCmd(void) {}

#endif /* CHASSIS_BALANCE */
