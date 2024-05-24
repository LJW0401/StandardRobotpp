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
#include "CAN_communication.h"
#include "bsp_delay.h"
#include "detect_task.h"
#include "leg_model.h"
#include "signal_generator.h"
#include "stdbool.h"
#include "string.h"
#include "usb_task.h"
#include "user_lib.h"

#define LOCATION_CONTROL

static Chassis_s CHASSIS = {
    .mode = CHASSIS_OFF,
    .state = CHASSIS_STATE_ERROR,
    .error_code = 0,
    .yaw_mid = 0,

    .upper_limit =
        {
            .theta = MAX_THETA,
            .theta_dot = MAX_THETA_DOT,
            .x = MAX_X,
            .x_dot = MAX_X_DOT,
            .phi = MAX_PHI,
            .phi_dot = MAX_PHI_DOT,

            .speed_integral = MAX_SPEED_INTEGRAL,
            .roll = MAX_ROLL,
            .roll_velocity = MAX_ROLL_VELOCITY,
            .yaw = MAX_YAW,
            .yaw_velocity = MAX_YAW_VELOCITY,

            .joint_l =
                {
                    .Angle = MAX_JOINT_ANGLE,
                },
            .joint_r =
                {
                    .Angle = MAX_JOINT_ANGLE,
                },

            .leg_l =
                {
                    .length = MAX_LEG_LENGTH,
                    .angle = MAX_LEG_ANGLE,
                    .dLength = 0.0f,
                    .dAngle = 0.0f,
                    .ddLength = 0.0f,
                    .last_dLength = 0.0f,
                },
            .leg_r =
                {
                    .length = MAX_LEG_LENGTH,
                    .angle = MAX_LEG_ANGLE,
                    .dLength = 0.0f,
                    .dAngle = 0.0f,
                    .ddLength = 0.0f,
                    .last_dLength = 0.0f,
                },
            .speed_vector =
                {
                    .vx = MAX_SPEED_VECTOR_VX,
                    .vy = MAX_SPEED_VECTOR_VY,
                    .wz = MAX_SPEED_VECTOR_WZ,
                },
        },

    .lower_limit =
        {
            .theta = MIN_THETA,
            .theta_dot = MIN_THETA_DOT,
            .x = MIN_X,
            .x_dot = MIN_X_DOT,
            .phi = MIN_PHI,
            .phi_dot = MIN_PHI_DOT,

            .speed_integral = MIN_SPEED_INTEGRAL,
            .roll = MIN_ROLL,
            .roll_velocity = MIN_ROLL_VELOCITY,
            .yaw = MIN_YAW,
            .yaw_velocity = MIN_YAW_VELOCITY,

            .joint_l =
                {
                    .Angle = MIN_JOINT_ANGLE,
                },
            .joint_r =
                {
                    .Angle = MIN_JOINT_ANGLE,
                },

            .leg_l =
                {
                    .length = MIN_LEG_LENGTH,
                    .angle = MIN_LEG_ANGLE,
                    .dLength = 0.0f,
                    .dAngle = 0.0f,
                    .ddLength = 0.0f,
                    .last_dLength = 0.0f,
                },
            .leg_r =
                {
                    .length = MIN_LEG_LENGTH,
                    .angle = MIN_LEG_ANGLE,
                    .dLength = 0.0f,
                    .dAngle = 0.0f,
                    .ddLength = 0.0f,
                    .last_dLength = 0.0f,
                },
            .speed_vector =
                {
                    .vx = MIN_SPEED_VECTOR_VX,
                    .vy = MIN_SPEED_VECTOR_VY,
                    .wz = MIN_SPEED_VECTOR_WZ,
                },
        },

    .ratio =
        {
            // clang-format off
            .k = {{1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f}, 
                  {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f}},
            // clang-format on
            .Tp = 1.0f,
            .T = 1.0f,
            .length = 1.0f,
        },
    .dyaw = 0.0f,
};

/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param[in]      none
 * @retval         none
 */
void ChassisInit(void)
{
    CHASSIS.rc = get_remote_control_point();  // 获取遥控器指针
    CHASSIS.imu = GetImuDataPoint();          // 获取IMU指针
    /*-------------------- 初始化底盘电机 --------------------*/
    for (uint8_t i = 0; i < 4; i++) {
        MotorInit(&CHASSIS.joint_motor[i], i + 1, JOINT_CAN, DM_8009, 1, 1, DM_MODE_MIT);
    }

    for (uint8_t i = 0; i < 2; i++) {
        MotorInit(&CHASSIS.wheel_motor[i], i + 1, WHEEL_CAN, MF_9025, 1, 1, 0);
    }

    /*-------------------- 值归零 --------------------*/
    memset(&CHASSIS.fdb, 0, sizeof(CHASSIS.fdb));
    memset(&CHASSIS.ref, 0, sizeof(CHASSIS.ref));

    /*-------------------- 初始化底盘PID --------------------*/
    float yaw_angle_pid[3] = {KP_CHASSIS_YAW_ANGLE, KI_CHASSIS_YAW_ANGLE, KD_CHASSIS_YAW_ANGLE};
    float yaw_velocity_pid[3] = {
        KP_CHASSIS_YAW_VELOCITY, KI_CHASSIS_YAW_VELOCITY, KD_CHASSIS_YAW_VELOCITY};
    float roll_angle_pid[3] = {KP_CHASSIS_ROLL_ANGLE, KI_CHASSIS_ROLL_ANGLE, KD_CHASSIS_ROLL_ANGLE};
    float roll_velocity_pid[3] = {
        KP_CHASSIS_ROLL_VELOCITY, KI_CHASSIS_ROLL_VELOCITY, KD_CHASSIS_ROLL_VELOCITY};
    float leg_length_length_pid[3] = {
        KP_CHASSIS_LEG_LENGTH_LENGTH, KI_CHASSIS_LEG_LENGTH_LENGTH, KD_CHASSIS_LEG_LENGTH_LENGTH};
    float leg_length_speed_pid[3] = {
        KP_CHASSIS_LEG_LENGTH_SPEED, KI_CHASSIS_LEG_LENGTH_SPEED, KD_CHASSIS_LEG_LENGTH_SPEED};
    float leg_angle_angle_pid[3] = {
        KP_CHASSIS_LEG_ANGLE_ANGLE, KI_CHASSIS_LEG_ANGLE_ANGLE, KD_CHASSIS_LEG_ANGLE_ANGLE};

    PID_init(
        &CHASSIS.pid.yaw_angle, PID_POSITION, yaw_angle_pid, MAX_OUT_CHASSIS_YAW_ANGLE,
        MAX_IOUT_CHASSIS_YAW_ANGLE);
    PID_init(
        &CHASSIS.pid.yaw_velocity, PID_POSITION, yaw_velocity_pid, MAX_OUT_CHASSIS_YAW_VELOCITY,
        MAX_IOUT_CHASSIS_YAW_VELOCITY);
    PID_init(
        &CHASSIS.pid.roll_angle, PID_POSITION, roll_angle_pid, MAX_OUT_CHASSIS_ROLL_ANGLE,
        MAX_IOUT_CHASSIS_ROLL_ANGLE);
    PID_init(
        &CHASSIS.pid.roll_velocity, PID_POSITION, roll_velocity_pid, MAX_OUT_CHASSIS_ROLL_VELOCITY,
        MAX_IOUT_CHASSIS_ROLL_VELOCITY);
    PID_init(
        &CHASSIS.pid.leg_length_left_length, PID_POSITION, leg_length_length_pid,
        MAX_OUT_CHASSIS_LEG_LENGTH_LENGTH, MAX_IOUT_CHASSIS_LEG_LENGTH_LENGTH);
    PID_init(
        &CHASSIS.pid.leg_length_left_speed, PID_POSITION, leg_length_speed_pid,
        MAX_OUT_CHASSIS_LEG_LENGTH_SPEED, MAX_IOUT_CHASSIS_LEG_LENGTH_SPEED);
    PID_init(
        &CHASSIS.pid.leg_length_right_length, PID_POSITION, leg_length_length_pid,
        MAX_OUT_CHASSIS_LEG_LENGTH_LENGTH, MAX_IOUT_CHASSIS_LEG_LENGTH_LENGTH);
    PID_init(
        &CHASSIS.pid.leg_length_right_speed, PID_POSITION, leg_length_speed_pid,
        MAX_OUT_CHASSIS_LEG_LENGTH_SPEED, MAX_IOUT_CHASSIS_LEG_LENGTH_SPEED);
    PID_init(
        &CHASSIS.pid.leg_angle_angle, PID_POSITION, leg_angle_angle_pid,
        MAX_OUT_CHASSIS_LEG_ANGLE_ANGLE, MAX_IOUT_CHASSIS_LEG_ANGLE_ANGLE);
}

/*-------------------- Handle exception --------------------*/

/**
 * @brief          异常处理
 * @param[in]      none
 * @retval         none
 */
void ChassisHandleException(void)
{
    if (toe_is_error(DBUS_TOE)) {
        CHASSIS.error_code |= DBUS_ERROR_OFFSET;
    } else {
        CHASSIS.error_code &= ~DBUS_ERROR_OFFSET;
    }
}

/*-------------------- Set mode --------------------*/

/**
 * @brief          设置模式
 * @param[in]      none
 * @retval         none
 */
void ChassisSetMode(void)
{
    if (CHASSIS.error_code & DBUS_ERROR_OFFSET) {  // 遥控器出错时的状态处理
        CHASSIS.mode = CHASSIS_OFF;
        return;
    }

    // if (CHASSIS.state == CHASSIS_STATE_ERROR) {  //底盘出错时的状态处理
    //     if ((CHASSIS.error_code | JOINT_ERROR_OFFSET) ||
    //         (CHASSIS.error_code | WHEEL_ERROR_OFFSET)) {
    //         CHASSIS.mode = CHASSIS_OFF;
    //     }
    //     return;
    // }

    if (switch_is_up(CHASSIS.rc->rc.s[CHASSIS_MODE_CHANNEL])) {
        CHASSIS.mode = CHASSIS_AUTO;
    } else if (switch_is_mid(CHASSIS.rc->rc.s[CHASSIS_MODE_CHANNEL])) {
        CHASSIS.mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    } else if (switch_is_down(CHASSIS.rc->rc.s[CHASSIS_MODE_CHANNEL])) {
        CHASSIS.mode = CHASSIS_FREE;
    }
}

/*-------------------- Observe --------------------*/

static void UpdateLegStatus(void);
static void UpdateJointStatus(void);
static void UpdateMotorStatus(void);

/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void ChassisObserver(void)
{
    UpdateMotorStatus();
    UpdateJointStatus();
    UpdateLegStatus();

    // 更新fdb数据
    CHASSIS.fdb.roll = CHASSIS.imu->roll;
    CHASSIS.fdb.roll_velocity = CHASSIS.imu->roll_vel;
    CHASSIS.fdb.yaw = CHASSIS.imu->yaw;
    CHASSIS.fdb.yaw_velocity = CHASSIS.imu->yaw_vel;

    // 更新LQR状态向量
    // clang-format off
    CHASSIS.fdb.theta = (CHASSIS.fdb.leg_l.angle + CHASSIS.fdb.leg_r.angle) / 2 
                        - M_PI_2 - CHASSIS.imu->pitch;
    CHASSIS.fdb.theta_dot = (CHASSIS.fdb.leg_l.dAngle + CHASSIS.fdb.leg_r.dAngle) / 2 
                            - CHASSIS.imu->pitch_vel;
    CHASSIS.fdb.x       = 0;
    CHASSIS.fdb.x_dot   = WHEEL_RADIUS * (CHASSIS.wheel_motor[0].fdb.vel + CHASSIS.wheel_motor[1].fdb.vel) / 2;
    CHASSIS.fdb.phi     = CHASSIS.imu->pitch;
    CHASSIS.fdb.phi_dot = CHASSIS.imu->pitch_vel;
    // clang-format on

    // CHASSIS.dyaw = (CHASSIS.yaw_motor.motor_measure->ecd * DJI_GM6020_ECD_TO_RAD - CHASSIS.yaw_mid);
    OutputPCData.packets[0].data = CHASSIS.wheel_motor[0].fdb.vel;
    OutputPCData.packets[1].data = CHASSIS.wheel_motor[0].fdb.pos;
    OutputPCData.packets[2].data = CHASSIS.wheel_motor[0].fdb.curr;
    OutputPCData.packets[3].data = CHASSIS.wheel_motor[0].fdb.temp;
    OutputPCData.packets[4].data = CHASSIS.joint_motor[0].set.pos;
    OutputPCData.packets[5].data = CHASSIS.joint_motor[0].fdb.pos;
    OutputPCData.packets[6].data = CHASSIS.joint_motor[1].fdb.pos;
    OutputPCData.packets[7].data = CHASSIS.joint_motor[2].fdb.pos;
    OutputPCData.packets[8].data = CHASSIS.joint_motor[3].fdb.pos;
    OutputPCData.packets[9].data = CHASSIS.imu->pitch;
    OutputPCData.packets[10].data = CHASSIS.imu->pitch_vel;
    OutputPCData.packets[11].data = CHASSIS.imu->roll;
    OutputPCData.packets[12].data = CHASSIS.imu->roll_vel;
    OutputPCData.packets[13].data = CHASSIS.imu->yaw;
    OutputPCData.packets[14].data = CHASSIS.imu->yaw_vel;
    OutputPCData.packets[15].data = CHASSIS.imu->x_accel;
    OutputPCData.packets[16].data = CHASSIS.imu->y_accel;
    OutputPCData.packets[17].data = CHASSIS.imu->z_accel;
}

/**
 * @brief 更新关节状态
 * @param  none
 */
static void UpdateJointStatus(void)
{
    CHASSIS.fdb.joint_l.Angle[0] = CHASSIS.joint_motor[0].fdb.pos;
    CHASSIS.fdb.joint_l.Angle[1] = CHASSIS.joint_motor[1].fdb.pos;
    CHASSIS.fdb.joint_r.Angle[0] = CHASSIS.joint_motor[2].fdb.pos;
    CHASSIS.fdb.joint_r.Angle[1] = CHASSIS.joint_motor[3].fdb.pos;

    CHASSIS.fdb.joint_l.dAngle[0] = CHASSIS.joint_motor[0].fdb.vel;
    CHASSIS.fdb.joint_l.dAngle[1] = CHASSIS.joint_motor[1].fdb.vel;
    CHASSIS.fdb.joint_r.dAngle[0] = CHASSIS.joint_motor[2].fdb.vel;
    CHASSIS.fdb.joint_r.dAngle[1] = CHASSIS.joint_motor[3].fdb.vel;
}

/**
 * @brief  更新底盘电机数据
 * @param  none
 */
static void UpdateMotorStatus(void)
{
    for (uint8_t i = 0; i < 4; i++) {
        GetMotorMeasure(&CHASSIS.joint_motor[i]);
    }

    for (uint8_t i = 0; i < 2; i++) {
        GetMotorMeasure(&CHASSIS.wheel_motor[i]);
    }
}

/**
 * @brief  更新腿部状态
 * @param  none
 */
static void UpdateLegStatus(void)
{
    double leg_pos[2];
    double leg_speed[2];
    /*-------------------- 更新左腿 --------------------*/
    // 更新位置信息
    LegFKine(CHASSIS.fdb.joint_l.Angle[1], CHASSIS.fdb.joint_l.Angle[0], leg_pos);
    CHASSIS.fdb.leg_l.length = leg_pos[0];
    CHASSIS.fdb.leg_l.angle = leg_pos[1];

    // 更新速度信息
    // clang-format off
    CHASSIS.fdb.leg_l.last_dLength = CHASSIS.fdb.leg_l.dLength;
    LegSpeed(
        CHASSIS.fdb.joint_l.dAngle[1], CHASSIS.fdb.joint_l.dAngle[0],
        CHASSIS.fdb.joint_l.Angle[1] , CHASSIS.fdb.joint_l.Angle[0],
        leg_speed);
    CHASSIS.fdb.leg_l.dLength = leg_speed[0];
    CHASSIS.fdb.leg_l.dAngle  = leg_speed[1];

    // 计算腿长加速度
    CHASSIS.fdb.leg_l.ddLength =
        ((CHASSIS.fdb.leg_l.dLength - CHASSIS.fdb.leg_l.last_dLength) * 1000 / 4) * LEG_DDLENGTH_LPF_RATIO
        + CHASSIS.fdb.leg_l.ddLength * (1 - LEG_DDLENGTH_LPF_RATIO);
    // clang-format on

    /*-------------------- 更新右腿 --------------------*/
    // 更新位置信息
    LegFKine(CHASSIS.fdb.joint_r.Angle[1], CHASSIS.fdb.joint_r.Angle[0], leg_pos);
    CHASSIS.fdb.leg_r.length = leg_pos[0];
    CHASSIS.fdb.leg_r.angle = leg_pos[1];

    // 更新速度信息
    // clang-format off
    CHASSIS.fdb.leg_r.last_dLength = CHASSIS.fdb.leg_r.dLength;
    LegSpeed(
        CHASSIS.fdb.joint_r.dAngle[1], CHASSIS.fdb.joint_r.dAngle[0],
        CHASSIS.fdb.joint_r.Angle[1] , CHASSIS.fdb.joint_r.Angle[0],
        leg_speed);
    CHASSIS.fdb.leg_r.dLength = leg_speed[0];
    CHASSIS.fdb.leg_r.dAngle  = leg_speed[1];

    // 计算腿长加速度
    CHASSIS.fdb.leg_r.ddLength =
        ((CHASSIS.fdb.leg_r.dLength - CHASSIS.fdb.leg_r.last_dLength) * 1000 / 4) * LEG_DDLENGTH_LPF_RATIO 
        + CHASSIS.fdb.leg_r.ddLength * (1 - LEG_DDLENGTH_LPF_RATIO);
    // clang-format on
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

    v_set.vx = rc_x * RC_TO_ONE * CHASSIS.upper_limit.speed_vector.vx;
    v_set.vy = rc_y * RC_TO_ONE * CHASSIS.upper_limit.speed_vector.vy;
    v_set.wz = rc_wz * RC_TO_ONE * CHASSIS.upper_limit.speed_vector.wz;
    switch (CHASSIS.mode) {
        case CHASSIS_FREE: {  // 底盘自由模式下，控制量为底盘坐标系下的速度
            break;
        }
        case CHASSIS_FOLLOW_GIMBAL_YAW: {  // 云台跟随模式下，控制量为云台坐标系下的速度，需要进行坐标转换
            v_set.vx = rc_x * RC_TO_ONE * CHASSIS.upper_limit.speed_vector.vx;
            v_set.vy = rc_y * RC_TO_ONE * CHASSIS.upper_limit.speed_vector.vy;
            v_set.wz = rc_wz * RC_TO_ONE * CHASSIS.upper_limit.speed_vector.wz;
            GimbalSpeedVectorToChassisSpeedVector(&v_set, CHASSIS.dyaw);
            break;
        }
        case CHASSIS_AUTO: {  // 底盘自动模式，控制量为云台坐标系下的速度，需要进行坐标转换
            break;
        }
        default:
            break;
    }

    CHASSIS.ref.speed_vector.vx = v_set.vx;
    CHASSIS.ref.speed_vector.vy = v_set.vy;
    CHASSIS.ref.speed_vector.wz = v_set.wz;

    float v = sqrtf(
        CHASSIS.ref.speed_vector.vx * CHASSIS.ref.speed_vector.vx +
        CHASSIS.ref.speed_vector.vy * CHASSIS.ref.speed_vector.vy);

    // clang-format off
    CHASSIS.ref.theta     = 0;
    CHASSIS.ref.theta_dot = 0;
    CHASSIS.ref.x         = 0;
    CHASSIS.ref.x_dot     = v;
    CHASSIS.ref.phi       = 0;
    CHASSIS.ref.phi_dot   = 0;
    // clang-format on

    float length = 0.2f;
    CHASSIS.ref.leg_l.length = length;
    CHASSIS.ref.leg_r.length = length;
}

/*-------------------- Console --------------------*/

static void LocomotionController(float Tp[2], float T_w[2]);
#ifdef LOCATION_CONTROL
static void LegController(double joint_pos_l[2], double joint_pos_r[2]);
#else
static void LegController(float F[2]);
#endif
static void SetK(float leg_length, float k[2][6]);
static void LQRFeedbackCalc(float k[2][6], float x[6], float t[2]);

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
            float tp[2], t[2];
            LocomotionController(tp, t);
#ifdef LOCATION_CONTROL
            double joint_pos_l[2], joint_pos_r[2];
            LegController(joint_pos_l, joint_pos_r);
#else
            LegController(float F[2]);
#endif
            break;
        }
    }

    for (uint8_t i = 0; i < 4; i++) {
        CHASSIS.joint_motor[i].set.pos = GenerateSinWave(1, 0, 2);
        CHASSIS.joint_motor[i].set.tor = 0;
        CHASSIS.joint_motor[i].set.vel = 0;
        CHASSIS.joint_motor[i].set.curr = 0;
    }

    for (uint8_t i = 0; i < 2; i++) {
        CHASSIS.wheel_motor[i].set.pos = 0;
        CHASSIS.wheel_motor[i].set.tor = GenerateSinWave(0.3, 0, 2);
        CHASSIS.wheel_motor[i].set.vel = 0;
        CHASSIS.wheel_motor[i].set.curr = 0;
    }
}

/**
 * @brief      运动控制器
 * @param[out]  Tp 输出的髋关节力矩
 * @param[out]  T_w 输出的驱动轮力矩
 */
static void LocomotionController(float Tp[2], float T_w[2])
{
    float x[6];

    x[0] = CHASSIS.fdb.theta - CHASSIS.ref.theta;
    x[1] = CHASSIS.fdb.theta_dot - CHASSIS.ref.theta_dot;
    x[2] = CHASSIS.fdb.x - CHASSIS.ref.x;
    x[3] = CHASSIS.fdb.x_dot - CHASSIS.ref.x_dot;
    x[4] = CHASSIS.fdb.phi - CHASSIS.ref.phi;
    x[5] = CHASSIS.fdb.phi_dot - CHASSIS.ref.phi_dot;

    float leg_length = (CHASSIS.fdb.leg_l.length + CHASSIS.fdb.leg_r.length) / 2;
    float k[2][6];
    SetK(leg_length, k);
    float t_tp[2];
    LQRFeedbackCalc(k, x, t_tp);
    float t = t_tp[0];
    float tp = t_tp[1];

    float dyaw;
    switch (CHASSIS.mode) {
        case CHASSIS_FOLLOW_GIMBAL_YAW: {
            dyaw = CHASSIS.dyaw;
            break;
        }
        default: {
            dyaw = CHASSIS.ref.yaw - CHASSIS.fdb.yaw;
            break;
        }
    }
    dyaw = theta_format(dyaw);
    PID_calc(&CHASSIS.pid.yaw_angle, dyaw, 0);
    PID_calc(&CHASSIS.pid.yaw_velocity, CHASSIS.fdb.yaw_velocity, CHASSIS.pid.yaw_angle.out);

    float dangle = CHASSIS.fdb.leg_l.angle - CHASSIS.fdb.leg_r.angle;
    PID_calc(&CHASSIS.pid.leg_angle_angle, dangle, 0);

    T_w[0] = t + CHASSIS.pid.yaw_velocity.out;
    T_w[1] = t - CHASSIS.pid.yaw_velocity.out;
    Tp[0] = tp + CHASSIS.pid.leg_angle_angle.out;
    Tp[1] = tp - CHASSIS.pid.leg_angle_angle.out;
}

/**
 * @brief         设置LQR的反馈矩阵K
 * @param[in]     leg_length 当前腿长
 * @param[out]    k 返回的反馈矩阵指针
 * @return        none
 */
static void SetK(float leg_length, float k[2][6])
{
    double k_res[12] = {0};
    L2K(leg_length, k_res);

    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 2; j++) {
            k[j][i] = k_res[i * 2 + j] * CHASSIS.ratio.k[j][i];
        }
    }
}

/**
 * @brief         矩阵相乘，计算LQR输出
 * @param[in]     k   LQR反馈矩阵K
 * @param[in]     x   状态变量向量
 * @param[out]    t 反馈数据T和Tp
 * @return        none
 * @note          T为t[0],Tp为t[1]
 */
static void LQRFeedbackCalc(float k[2][6], float x[6], float t[2])
{
    t[0] = k[0][0] * x[0] + k[0][1] * x[1] + k[0][2] * x[2] + k[0][3] * x[3] + k[0][4] * x[4] +
           k[0][5] * x[5];
    t[1] = k[1][0] * x[0] + k[1][1] * x[1] + k[1][2] * x[2] + k[1][3] * x[3] + k[1][4] * x[4] +
           k[1][5] * x[5];
}
#ifdef LOCATION_CONTROL
/**
 * @brief 腿部控制器
 * @param[out]  joint_pos_l 左关节电机设定位置 0-后，1-前
 * @param[out]  joint_pos_r 右关节电机设定位置 0-后，1-前
 */
static void LegController(double joint_pos_l[2], double joint_pos_r[2])
{
    LegIKine(CHASSIS.ref.leg_l.length, CHASSIS.ref.leg_l.angle,
             joint_pos_l);  // 计算左关节摆角

    LegIKine(CHASSIS.ref.leg_r.length, CHASSIS.ref.leg_r.angle,
             joint_pos_r);  // 计算右关节摆角
}
#else
/**
 * @brief 腿部控制器
 * @param[out]  F 输出的腿部沿杆方向的力F
 */
static void LegController(float F[2])
{
    PID_calc(
        &CHASSIS.pid.leg_length_left_length, CHASSIS.fdb.leg_l.length, CHASSIS.ref.leg_l.length);
    float theta_l = CHASSIS.fdb.leg_l.angle - M_PI_2 - CHASSIS.imu.pitch;
    float fdf_left = LegFeedforward(CHASSIS.fdb.x[0]);

    PID_calc(
        &CHASSIS.pid.leg_length_right_length, CHASSIS.fdb.leg_r.length, CHASSIS.ref.leg_r.length);
    float theta_r = CHASSIS.fdb.leg_r.angle - M_PI_2 - CHASSIS.imu.pitch;
    float fdf_right = LegFeedforward(CHASSIS.fdb.x[1]);

    PID_calc(&CHASSIS.pid.roll_angle, CHASSIS.fdb.roll, CHASSIS.ref.roll);
    PID_calc(&CHASSIS.pid.roll_velocity, CHASSIS.fdb.roll_velocity, CHASSIS.pid.roll_angle.out);

    F[0] = CHASSIS.pid.leg_length_left_length.out + fdf_left + CHASSIS.pid.roll_velocity.out;
    F[1] = CHASSIS.pid.leg_length_right_length.out + fdf_right - CHASSIS.pid.roll_velocity.out;
}
#endif

/*-------------------- Cmd --------------------*/
static void SendJointMotorCmd(void);
static void SendWheelMotorCmd(void);

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */
void ChassisSendCmd(void)
{
    SendJointMotorCmd();
    SendWheelMotorCmd();
}

/**
 * @brief 发送关节电机控制指令
 * @param[in] chassis
 */
static void SendJointMotorCmd(void)
{
    uint8_t cnt;
    if (CHASSIS.mode == CHASSIS_OFF) {
        DmMitStop(&CHASSIS.joint_motor[0]);
        DmMitStop(&CHASSIS.joint_motor[1]);
        delay_us(200);
        DmMitStop(&CHASSIS.joint_motor[2]);
        DmMitStop(&CHASSIS.joint_motor[3]);
    } else {
        bool flag = false;
        for (uint8_t i = 0; i < 4; i++) {
            if (cnt % 2 == 0) {
                delay_us(200);
            }
            if (CHASSIS.joint_motor[i].fdb.state == DM_STATE_DISABLE) {
                DmEnable(&CHASSIS.joint_motor[i]);
                flag = true;
                cnt++;
            }
        }

        if (flag) {
            delay_us(200);
        }

        DmMitCtrlPosition(&CHASSIS.joint_motor[0], 2, 1);
        DmMitCtrlPosition(&CHASSIS.joint_motor[1], 2, 1);
        delay_us(200);
        DmMitCtrlPosition(&CHASSIS.joint_motor[2], 2, 1);
        DmMitCtrlPosition(&CHASSIS.joint_motor[3], 2, 1);
    }
}

/**
 * @brief 发送驱动轮电机控制指令
 * @param chassis
 */
static void SendWheelMotorCmd(void)
{
    if (CHASSIS.mode == CHASSIS_OFF) {
        LkMultipleTorqueControl(WHEEL_CAN, 0, 0, 0, 0);
    } else {
        // clang-format off
        LkMultipleTorqueControl(WHEEL_CAN,
            CHASSIS.wheel_motor[0].set.tor, CHASSIS.wheel_motor[1].set.tor, 0, 0);
        // clang-format on
    }
}

#endif /* CHASSIS_BALANCE */
