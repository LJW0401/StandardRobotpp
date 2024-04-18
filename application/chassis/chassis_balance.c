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
          .vx = MAX_SPEED_VECTOR_VX,
          .vy = MAX_SPEED_VECTOR_VY,
          .wz = MAX_SPEED_VECTOR_WZ,
        },
      .roll = MAX_ROLL,
      .yaw = MAX_YAW,
      .x = {MAX_X_0, MAX_X_1, MAX_X_2, MAX_X_3, MAX_X_4, MAX_X_5},
      .leg_length = MAX_LEG_LENGTH,
    },
  .lower_limit =
    {
      .speed_vector =
        {
          .vx = MIN_SPEED_VECTOR_VX,
          .vy = MIN_SPEED_VECTOR_VY,
          .wz = MIN_SPEED_VECTOR_WZ,
        },
      .roll = MIN_ROLL,
      .yaw = MIN_YAW,
      .x = {MIN_X_0, MIN_X_1, MIN_X_2, MIN_X_3, MIN_X_4, MIN_X_5},
      .leg_length = MIN_LEG_LENGTH,
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
  .imu =
    {
      .yaw = 0.0f,
      .pitch = 0.0f,
      .roll = 0.0f,
      .pitch_velocity = 0.0f,
      .roll_velocity = 0.0f,
      .yaw_velocity = 0.0f,
      .xAccel = 0.0f,
      .yAccel = 0.0f,
      .zAccel = 0.0f,
    },
  .pid =
    {
      .yaw_pid_angle =
        {
          .Kp = KP_CHASSIS_YAW_ANGLE,
          .Ki = KI_CHASSIS_YAW_ANGLE,
          .Kd = KD_CHASSIS_YAW_ANGLE,
          .max_iout = MAX_IOUT_CHASSIS_YAW_ANGLE,
          .max_out = MAX_OUT_CHASSIS_YAW_ANGLE,
        },
      .yaw_pid_velocity =
        {
          .Kp = KP_CHASSIS_YAW_VELOCITY,
          .Ki = KI_CHASSIS_YAW_VELOCITY,
          .Kd = KD_CHASSIS_YAW_VELOCITY,
          .max_iout = MAX_IOUT_CHASSIS_YAW_VELOCITY,
          .max_out = MAX_OUT_CHASSIS_YAW_VELOCITY,
        },
      .roll_pid_angle =
        {
          .Kp = KP_CHASSIS_ROLL_ANGLE,
          .Ki = KI_CHASSIS_ROLL_ANGLE,
          .Kd = KD_CHASSIS_ROLL_ANGLE,
          .max_iout = MAX_IOUT_CHASSIS_ROLL_ANGLE,
          .max_out = MAX_OUT_CHASSIS_ROLL_ANGLE,
        },
      .roll_pid_velocity =
        {
          .Kp = KP_CHASSIS_ROLL_VELOCITY,
          .Ki = KI_CHASSIS_ROLL_VELOCITY,
          .Kd = KD_CHASSIS_ROLL_VELOCITY,
          .max_iout = MAX_IOUT_CHASSIS_ROLL_VELOCITY,
          .max_out = MAX_OUT_CHASSIS_ROLL_VELOCITY,
        },
      .leg_length_pid_length =
        {
          .Kp = KP_CHASSIS_LEG_LENGTH_LENGTH,
          .Ki = KI_CHASSIS_LEG_LENGTH_LENGTH,
          .Kd = KD_CHASSIS_LEG_LENGTH_LENGTH,
          .max_iout = MAX_IOUT_CHASSIS_LEG_LENGTH_LENGTH,
          .max_out = MAX_OUT_CHASSIS_LEG_LENGTH_LENGTH,
        },
      .leg_length_pid_speed =
        {
          .Kp = KP_CHASSIS_LEG_LENGTH_SPEED,
          .Ki = KI_CHASSIS_LEG_LENGTH_SPEED,
          .Kd = KD_CHASSIS_LEG_LENGTH_SPEED,
          .max_iout = MAX_IOUT_CHASSIS_LEG_LENGTH_SPEED,
          .max_out = MAX_OUT_CHASSIS_LEG_LENGTH_SPEED,
        },
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
    double left_leg_pos[2];
    double right_leg_pos[2];
    LegFKine(0, 0, left_leg_pos);   // 获取五连杆等效连杆长度
    LegFKine(0, 0, right_leg_pos);  // 获取五连杆等效连杆长度

    // 更新底盘IMU数据
    CHASSIS.imu.yaw = get_INS_angle_point()[0];
    CHASSIS.imu.pitch = get_INS_angle_point()[1];
    CHASSIS.imu.roll = get_INS_angle_point()[2];

    CHASSIS.imu.roll_velocity = get_gyro_data_point()[0];
    CHASSIS.imu.pitch_velocity = get_gyro_data_point()[1];
    CHASSIS.imu.yaw_velocity = get_gyro_data_point()[2];

    CHASSIS.imu.xAccel = get_accel_data_point()[0];
    CHASSIS.imu.yAccel = get_accel_data_point()[1];
    CHASSIS.imu.zAccel = get_accel_data_point()[2];

    // 更新LQR状态向量
    CHASSIS.feedback.x[0] = CHASSIS.imu.pitch;
    CHASSIS.feedback.x[1] = CHASSIS.imu.pitch_velocity;
    CHASSIS.feedback.x[2] = 0;
    CHASSIS.feedback.x[3] =
      WHEEL_RADIUS;  //(left_wheel.speed + right_wheel.speed) / 2 * WHEEL_RADIUS;
    CHASSIS.feedback.x[4] =
      (CHASSIS.feedback.leg_pos[0].angle + CHASSIS.feedback.leg_pos[1].angle) /
        2 -
      M_PI_2 - CHASSIS.imu.pitch;
    CHASSIS.feedback.x[5] = 0;

    CHASSIS.dyaw =
      (CHASSIS.yaw_motor->motor_measure->ecd * DJI_GM6020_ECD_TO_RAD -
       CHASSIS.yaw_mid);
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
    rc_deadband_limit(
      CHASSIS.rc->rc.ch[CHASSIS_X_CHANNEL], rc_x, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(
      CHASSIS.rc->rc.ch[CHASSIS_Y_CHANNEL], rc_y, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(
      CHASSIS.rc->rc.ch[CHASSIS_WZ_CHANNEL], rc_wz, CHASSIS_RC_DEADLINE);

    ChassisSpeedVector_t v_set = {0.0f, 0.0f, 0.0f};
    if (
      CHASSIS.mode ==
      CHASSIS_FREE)  // 底盘自由模式下，控制量为底盘坐标系下的速度
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
      CHASSIS.mode ==
      CHASSIS_AUTO)  // 底盘自动模式，控制量为云台坐标系下的速度，需要进行坐标转换
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
        default: {
            float x[6];
            uint8_t i;
            for (i = 0; i < 6; i++) {
                x[i] = CHASSIS.feedback.x[i] - CHASSIS.reference.x[i];
            }
            break;
        }
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
