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

#define CALIBRATE_STOP_VELOCITY 0.05f  // rad/s
#define CALIBRATE_STOP_TIME 200        // ms
#define CALIBRATE_VELOCITY 2.0f        // rad/s

static Calibrate_s CALIBRATE = {
    .cali_cnt = 0,
    .velocity = {0.0f, 0.0f, 0.0f, 0.0f},
    .stpo_time = {0, 0, 0, 0},
    .reached = {false, false, false, false},
    .calibrated = false,
};

static Chassis_s CHASSIS = {
    .mode = CHASSIS_OFF,
    .state = CHASSIS_STATE_ERROR,
    .error_code = 0,
    .yaw_mid = 0,

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
    MotorInit(&CHASSIS.joint_motor[0], 1, JOINT_CAN, DM_8009, J0_DIRECTION, 1, DM_MODE_MIT);
    MotorInit(&CHASSIS.joint_motor[1], 2, JOINT_CAN, DM_8009, J1_DIRECTION, 1, DM_MODE_MIT);
    MotorInit(&CHASSIS.joint_motor[2], 3, JOINT_CAN, DM_8009, J2_DIRECTION, 1, DM_MODE_MIT);
    MotorInit(&CHASSIS.joint_motor[3], 4, JOINT_CAN, DM_8009, J3_DIRECTION, 1, DM_MODE_MIT);

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

    for (uint8_t i = 0; i < 4; i++) {
        if (fabs(CHASSIS.joint_motor[i].fdb.tor) > 5.0f) {
            CHASSIS.error_code |= JOINT_ERROR_OFFSET;
            break;
        } else {
            CHASSIS.error_code &= ~JOINT_ERROR_OFFSET;
        }
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
        CHASSIS.mode = CHASSIS_ZERO_FORCE;
        return;
    }

    if (CHASSIS.error_code & JOINT_ERROR_OFFSET) {  // 关节电机出错时的状态处理
        CHASSIS.mode = CHASSIS_ZERO_FORCE;
        return;
    }

    if (CHASSIS.mode == CHASSIS_CALIBRATE && (!CALIBRATE.calibrated)) {  //校准完成后才退出校准
        return;
    }

    if (switch_is_down(CHASSIS.rc->rc.s[0]) && switch_is_down(CHASSIS.rc->rc.s[1]) &&
        CALIBRATE.cali_cnt > 100) {  // 切入底盘校准
        CHASSIS.mode = CHASSIS_CALIBRATE;
        CALIBRATE.calibrated = false;

        uint32_t now = HAL_GetTick();
        for (uint8_t i = 0; i < 4; i++) {
            CALIBRATE.reached[i] = false;
            CALIBRATE.stpo_time[i] = now;
        }

        return;
    }

    if (switch_is_up(CHASSIS.rc->rc.s[CHASSIS_MODE_CHANNEL])) {
        CHASSIS.mode = CHASSIS_FREE;
    } else if (switch_is_mid(CHASSIS.rc->rc.s[CHASSIS_MODE_CHANNEL])) {
        CHASSIS.mode = CHASSIS_DEBUG;  // use for test, delete when release
    } else if (switch_is_down(CHASSIS.rc->rc.s[CHASSIS_MODE_CHANNEL])) {
        CHASSIS.mode = CHASSIS_ZERO_FORCE;
    }
}

/*-------------------- Observe --------------------*/

#define ZERO_POS_THRESHOLD 0.001f

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

    // 更新校准相关的数据
    if ((CHASSIS.rc->rc.ch[0] < -655) && (CHASSIS.rc->rc.ch[1] < -655) &&
        (CHASSIS.rc->rc.ch[2] > 655) && (CHASSIS.rc->rc.ch[3] < -655)) {
        CALIBRATE.cali_cnt++;  // 遥控器下内八进入底盘校准
    } else {
        CALIBRATE.cali_cnt = 0;
    }

    if ((CHASSIS.mode == CHASSIS_CALIBRATE) &&
        fabs(CHASSIS.joint_motor[0].fdb.pos) < ZERO_POS_THRESHOLD &&
        fabs(CHASSIS.joint_motor[1].fdb.pos) < ZERO_POS_THRESHOLD &&
        fabs(CHASSIS.joint_motor[2].fdb.pos) < ZERO_POS_THRESHOLD &&
        fabs(CHASSIS.joint_motor[3].fdb.pos) < ZERO_POS_THRESHOLD) {
        CALIBRATE.calibrated = true;
    }

    // 更新fdb数据
    CHASSIS.fdb.roll = CHASSIS.imu->roll;
    CHASSIS.fdb.roll_velocity = CHASSIS.imu->roll_vel;
    CHASSIS.fdb.yaw = CHASSIS.imu->yaw;
    CHASSIS.fdb.yaw_velocity = CHASSIS.imu->yaw_vel;

    // 更新LQR状态向量
    // clang-format off
    CHASSIS.fdb.theta = (CHASSIS.fdb.leg[0].rod.Angle + CHASSIS.fdb.leg[1].rod.Angle) / 2 
                        - M_PI_2 - CHASSIS.imu->pitch;
    CHASSIS.fdb.theta_dot = (CHASSIS.fdb.leg[0].rod.dAngle + CHASSIS.fdb.leg[1].rod.dAngle) / 2 
                            - CHASSIS.imu->pitch_vel;
    CHASSIS.fdb.x       = 0;
    // TODO: 检测速度方向是否正确
    CHASSIS.fdb.x_dot   = WHEEL_RADIUS * (CHASSIS.wheel_motor[0].fdb.vel + CHASSIS.wheel_motor[1].fdb.vel) / 2;
    CHASSIS.fdb.phi     = CHASSIS.imu->pitch;
    CHASSIS.fdb.phi_dot = CHASSIS.imu->pitch_vel;
    // clang-format on

    uint32_t now = HAL_GetTick();
    if (CHASSIS.mode == CHASSIS_CALIBRATE) {
        for (uint8_t i = 0; i < 4; i++) {
            CALIBRATE.velocity[i] = CHASSIS.joint_motor[i].fdb.vel;
            if (CALIBRATE.velocity[i] > CALIBRATE_STOP_VELOCITY) {  // 速度大于阈值时重置计时
                CALIBRATE.reached[i] = false;
                CALIBRATE.stpo_time[i] = now;
            } else {
                if (now - CALIBRATE.stpo_time[i] > CALIBRATE_STOP_TIME) {
                    CALIBRATE.reached[i] = true;
                }
            }
        }
    }

    OutputPCData.packets[0].data = CHASSIS.joint_motor[0].fdb.pos;
    OutputPCData.packets[1].data = CHASSIS.joint_motor[1].fdb.pos;
    OutputPCData.packets[2].data = CHASSIS.joint_motor[2].fdb.pos;
    OutputPCData.packets[3].data = CHASSIS.joint_motor[3].fdb.pos;
    OutputPCData.packets[4].data = CHASSIS.joint_motor[0].fdb.tor;
    OutputPCData.packets[5].data = CHASSIS.joint_motor[1].fdb.tor;
    OutputPCData.packets[6].data = CHASSIS.joint_motor[2].fdb.tor;
    OutputPCData.packets[7].data = CHASSIS.joint_motor[3].fdb.tor;
    OutputPCData.packets[8].data = CHASSIS.joint_motor[0].set.pos;
    OutputPCData.packets[9].data = CHASSIS.joint_motor[1].set.pos;
    OutputPCData.packets[10].data = CHASSIS.joint_motor[2].set.pos;
    OutputPCData.packets[11].data = CHASSIS.joint_motor[3].set.pos;
    OutputPCData.packets[12].data = CHASSIS.fdb.leg[0].rod.Length;
    OutputPCData.packets[13].data = CHASSIS.fdb.leg[0].rod.Angle;
    OutputPCData.packets[14].data = CHASSIS.fdb.leg[1].rod.Length;
    OutputPCData.packets[15].data = CHASSIS.fdb.leg[1].rod.Angle;
    // OutputPCData.packets[16].data = CHASSIS.rc->rc.ch[0];
    // OutputPCData.packets[17].data = CHASSIS.rc->rc.ch[1];
    // OutputPCData.packets[18].data = CHASSIS.rc->rc.ch[2];
    // OutputPCData.packets[19].data = CHASSIS.rc->rc.ch[3];
    // OutputPCData.packets[20].data = CHASSIS.mode;
}

/**
 * @brief 更新关节状态
 * @param  none
 */
static void UpdateJointStatus(void)
{
    CHASSIS.fdb.leg[0].joint.Angle[0] =
        theta_transform(CHASSIS.joint_motor[0].fdb.pos, J0_ANGLE_OFFSET, J0_DIRECTION, 1);
    CHASSIS.fdb.leg[0].joint.Angle[1] =
        theta_transform(CHASSIS.joint_motor[1].fdb.pos, J1_ANGLE_OFFSET, J1_DIRECTION, 1);
    CHASSIS.fdb.leg[1].joint.Angle[0] =
        theta_transform(CHASSIS.joint_motor[2].fdb.pos, J2_ANGLE_OFFSET, J2_DIRECTION, 1);
    CHASSIS.fdb.leg[1].joint.Angle[1] =
        theta_transform(CHASSIS.joint_motor[3].fdb.pos, J3_ANGLE_OFFSET, J3_DIRECTION, 1);

    CHASSIS.fdb.leg[0].joint.dAngle[0] = CHASSIS.joint_motor[0].fdb.vel;
    CHASSIS.fdb.leg[0].joint.dAngle[1] = CHASSIS.joint_motor[1].fdb.vel;
    CHASSIS.fdb.leg[1].joint.dAngle[0] = CHASSIS.joint_motor[2].fdb.vel;
    CHASSIS.fdb.leg[1].joint.dAngle[1] = CHASSIS.joint_motor[3].fdb.vel;
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

    for (uint8_t i = 0; i < 2; i++) {
        // 更新位置信息
        LegFKine(CHASSIS.fdb.leg[i].joint.Angle[1], CHASSIS.fdb.leg[i].joint.Angle[0], leg_pos);
        CHASSIS.fdb.leg[i].rod.Length = leg_pos[0];
        CHASSIS.fdb.leg[i].rod.Angle = leg_pos[1];

        // 更新速度信息
        // clang-format off
        LegSpeed(
            CHASSIS.fdb.leg[i].joint.dAngle[1], CHASSIS.fdb.leg[i].joint.dAngle[0],
            CHASSIS.fdb.leg[i].joint.Angle[1] , CHASSIS.fdb.leg[i].joint.Angle[0],
            leg_speed);
        CHASSIS.fdb.leg[i].rod.dLength = leg_speed[0];
        CHASSIS.fdb.leg[i].rod.dAngle  = leg_speed[1];
        // clang-format on
    }
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

    v_set.vx = rc_x * RC_TO_ONE * MAX_SPEED_VECTOR_VX;
    v_set.vy = rc_y * RC_TO_ONE * MAX_SPEED_VECTOR_VY;
    v_set.wz = rc_wz * RC_TO_ONE * MAX_SPEED_VECTOR_WZ;
    switch (CHASSIS.mode) {
        case CHASSIS_FREE: {  // 底盘自由模式下，控制量为底盘坐标系下的速度
            break;
        }
        case CHASSIS_FOLLOW_GIMBAL_YAW: {  // 云台跟随模式下，控制量为云台坐标系下的速度，需要进行坐标转换
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

    // TODO:用于测试，后续删除
    float length = CHASSIS.rc->rc.ch[1] * RC_TO_ONE * (MAX_LEG_LENGTH - MIN_LEG_LENGTH) / 2 +
                   (MAX_LEG_LENGTH + MIN_LEG_LENGTH) / 2;
    CHASSIS.ref.leg[0].rod.Length = length;
    CHASSIS.ref.leg[1].rod.Length = length;

    float angle = CHASSIS.rc->rc.ch[3] * RC_TO_ONE * M_PI / 6 + M_PI_2;
    CHASSIS.ref.leg[0].rod.Angle = angle;
    CHASSIS.ref.leg[1].rod.Angle = angle;

    OutputPCData.packets[20].data = length;
    OutputPCData.packets[21].data = angle;
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

static void ConsoleZeroForce(void);
static void ConsoleDebug(void);
static void ConsoleCalibrate(void);
static void ConsoleNormal(void);

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void ChassisConsole(void)
{
    switch (CHASSIS.mode) {
        case CHASSIS_CALIBRATE: {
            ConsoleCalibrate();
        } break;
        case CHASSIS_FOLLOW_GIMBAL_YAW:
        case CHASSIS_STOP:
        case CHASSIS_SPIN:
        case CHASSIS_FREE: {
            ConsoleNormal();
        } break;
        case CHASSIS_DEBUG: {
            ConsoleDebug();
        } break;
        case CHASSIS_OFF:
        case CHASSIS_ZERO_FORCE:
        default: {
            ConsoleZeroForce();
        }
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

    float leg_length = (CHASSIS.fdb.leg[0].rod.Length + CHASSIS.fdb.leg[1].rod.Length) / 2;
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

    float dangle = CHASSIS.fdb.leg[0].rod.Angle - CHASSIS.fdb.leg[1].rod.Angle;
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
    LegIKine(CHASSIS.ref.leg[0].rod.Length, CHASSIS.ref.leg[0].rod.Angle, joint_pos_l);
    LegIKine(CHASSIS.ref.leg[1].rod.Length, CHASSIS.ref.leg[1].rod.Angle, joint_pos_r);
}
#else
/**
 * @brief 腿部控制器
 * @param[out]  F 输出的腿部沿杆方向的力F
 */
static void LegController(float F[2])
{
    PID_calc(
        &CHASSIS.pid.leg_length_left_length, CHASSIS.fdb.leg[0].rod.Length,
        CHASSIS.ref.leg[0].rod.Length);
    float theta_l = CHASSIS.fdb.leg[0].rod.Angle - M_PI_2 - CHASSIS.imu->pitch;
    float fdf_left = LegFeedforward(theta_l);

    PID_calc(
        &CHASSIS.pid.leg_length_right_length, CHASSIS.fdb.leg[1].rod.Length,
        CHASSIS.ref.leg[1].rod.Length);
    float theta_r = CHASSIS.fdb.leg[1].rod.Angle - M_PI_2 - CHASSIS.imu->pitch;
    float fdf_right = LegFeedforward(theta_r);

    // PID_calc(&CHASSIS.pid.roll_angle, CHASSIS.fdb.roll, CHASSIS.ref.roll);
    // PID_calc(&CHASSIS.pid.roll_velocity, CHASSIS.fdb.roll_velocity, CHASSIS.pid.roll_angle.out);

    F[0] = CHASSIS.pid.leg_length_left_length.out + fdf_left + CHASSIS.pid.roll_velocity.out;
    F[1] = CHASSIS.pid.leg_length_right_length.out + fdf_right - CHASSIS.pid.roll_velocity.out;
}
#endif

//* 各个模式下的控制

static void ConsoleZeroForce(void)
{
    CHASSIS.joint_motor[0].set.tor = 0;
    CHASSIS.joint_motor[1].set.tor = 0;
    CHASSIS.joint_motor[2].set.tor = 0;
    CHASSIS.joint_motor[3].set.tor = 0;

    CHASSIS.joint_motor[0].set.vel = 0;
    CHASSIS.joint_motor[1].set.vel = 0;
    CHASSIS.joint_motor[2].set.vel = 0;
    CHASSIS.joint_motor[3].set.vel = 0;

    CHASSIS.wheel_motor[0].set.tor = 0;
    CHASSIS.wheel_motor[0].set.tor = 0;
}

static void ConsoleCalibrate(void)
{
    CHASSIS.joint_motor[0].set.vel = -CALIBRATE_VELOCITY;
    CHASSIS.joint_motor[1].set.vel = CALIBRATE_VELOCITY;
    CHASSIS.joint_motor[2].set.vel = CALIBRATE_VELOCITY;
    CHASSIS.joint_motor[3].set.vel = -CALIBRATE_VELOCITY;

    CHASSIS.wheel_motor[0].set.tor = 0;
    CHASSIS.wheel_motor[0].set.tor = 0;
}

static void ConsoleDebug(void)
{
    CHASSIS.joint_motor[0].set.vel = CHASSIS.rc->rc.ch[0] * RC_TO_ONE;
    CHASSIS.joint_motor[1].set.vel = CHASSIS.rc->rc.ch[1] * RC_TO_ONE;
    CHASSIS.joint_motor[2].set.vel = CHASSIS.rc->rc.ch[2] * RC_TO_ONE;
    CHASSIS.joint_motor[3].set.vel = CHASSIS.rc->rc.ch[3] * RC_TO_ONE;

    CHASSIS.wheel_motor[0].set.tor = CHASSIS.rc->rc.ch[4] * RC_TO_ONE * 200;
    CHASSIS.wheel_motor[1].set.tor = CHASSIS.rc->rc.ch[4] * RC_TO_ONE * 200;
}

static void ConsoleNormal(void)
{
    float tp[2], t[2];
    LocomotionController(tp, t);
#ifdef LOCATION_CONTROL
    double joint_pos_l[2], joint_pos_r[2];
    LegController(joint_pos_l, joint_pos_r);

    // 当解算出的角度正常时，设置目标角度
    if (!(isnan(joint_pos_l[0]) || isnan(joint_pos_l[1]) || isnan(joint_pos_r[0]) ||
          isnan(joint_pos_r[1]))) {
        CHASSIS.joint_motor[0].set.pos =
            theta_transform(joint_pos_l[1], -J0_ANGLE_OFFSET, J0_DIRECTION, 1);
        CHASSIS.joint_motor[1].set.pos =
            theta_transform(joint_pos_l[0], -J1_ANGLE_OFFSET, J1_DIRECTION, 1);
        CHASSIS.joint_motor[2].set.pos =
            theta_transform(joint_pos_r[1], -J2_ANGLE_OFFSET, J2_DIRECTION, 1);
        CHASSIS.joint_motor[3].set.pos =
            theta_transform(joint_pos_r[0], -J3_ANGLE_OFFSET, J3_DIRECTION, 1);

        // 检测设定角度是否超过电机角度限制
        CHASSIS.joint_motor[0].set.pos =
            fp32_constrain(CHASSIS.joint_motor[0].set.pos, MIN_J0_ANGLE, MAX_J0_ANGLE);
        CHASSIS.joint_motor[1].set.pos =
            fp32_constrain(CHASSIS.joint_motor[1].set.pos, MIN_J1_ANGLE, MAX_J1_ANGLE);
        CHASSIS.joint_motor[2].set.pos =
            fp32_constrain(CHASSIS.joint_motor[2].set.pos, MIN_J2_ANGLE, MAX_J2_ANGLE);
        CHASSIS.joint_motor[3].set.pos =
            fp32_constrain(CHASSIS.joint_motor[3].set.pos, MIN_J3_ANGLE, MAX_J3_ANGLE);
    }

    // OutputPCData.packets[12].data = CHASSIS.joint_motor[0].set.pos;
    // OutputPCData.packets[13].data = CHASSIS.joint_motor[1].set.pos;
    // OutputPCData.packets[14].data = CHASSIS.joint_motor[2].set.pos;
    // OutputPCData.packets[15].data = CHASSIS.joint_motor[3].set.pos;
    // OutputPCData.packets[16].data = joint_pos_l[1];
    // OutputPCData.packets[17].data = joint_pos_l[0];
    // OutputPCData.packets[18].data = joint_pos_r[1];
    // OutputPCData.packets[19].data = joint_pos_r[0];
#else
    float F[2];
    LegController(F);
#endif

    CHASSIS.joint_motor[0].set.vel = 0;
    CHASSIS.joint_motor[1].set.vel = 0;
    CHASSIS.joint_motor[2].set.vel = 0;
    CHASSIS.joint_motor[3].set.vel = 0;
}

/*-------------------- Cmd --------------------*/

#define CALIBRATE_VEL_KP 4.0f
#define DEBUG_VEL_KP 4.0f
#define ZERO_FORCE_VEL_KP 1.0f

#define NORMAL_POS_KP 20.0f
#define NORMAL_POS_KD 1.5f

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

        switch (CHASSIS.mode) {
            case CHASSIS_FOLLOW_GIMBAL_YAW:
            case CHASSIS_STOP:
            case CHASSIS_SPIN:
            case CHASSIS_FREE: {
#ifdef LOCATION_CONTROL
                DmMitCtrlPosition(&CHASSIS.joint_motor[0], NORMAL_POS_KP, NORMAL_POS_KD);
                DmMitCtrlPosition(&CHASSIS.joint_motor[1], NORMAL_POS_KP, NORMAL_POS_KD);
                delay_us(200);
                DmMitCtrlPosition(&CHASSIS.joint_motor[2], NORMAL_POS_KP, NORMAL_POS_KD);
                DmMitCtrlPosition(&CHASSIS.joint_motor[3], NORMAL_POS_KP, NORMAL_POS_KD);
#else
                DmMitCtrlTorque(&CHASSIS.joint_motor[0]);
                DmMitCtrlTorque(&CHASSIS.joint_motor[1]);
                delay_us(200);
                DmMitCtrlTorque(&CHASSIS.joint_motor[2]);
                DmMitCtrlTorque(&CHASSIS.joint_motor[3]);
#endif
            } break;
            case CHASSIS_CALIBRATE: {
                DmMitCtrlVelocity(&CHASSIS.joint_motor[0], CALIBRATE_VEL_KP);
                DmMitCtrlVelocity(&CHASSIS.joint_motor[1], CALIBRATE_VEL_KP);
                delay_us(200);
                DmMitCtrlVelocity(&CHASSIS.joint_motor[2], CALIBRATE_VEL_KP);
                DmMitCtrlVelocity(&CHASSIS.joint_motor[3], CALIBRATE_VEL_KP);

                if (CALIBRATE.reached[0] && CALIBRATE.reached[1] && CALIBRATE.reached[2] &&
                    CALIBRATE.reached[3]) {
                    delay_us(200);
                    DmSavePosZero(&CHASSIS.joint_motor[0]);
                    DmSavePosZero(&CHASSIS.joint_motor[1]);
                    delay_us(200);
                    DmSavePosZero(&CHASSIS.joint_motor[2]);
                    DmSavePosZero(&CHASSIS.joint_motor[3]);
                }
            } break;
            case CHASSIS_DEBUG: {
                DmMitCtrlVelocity(&CHASSIS.joint_motor[0], DEBUG_VEL_KP);
                DmMitCtrlVelocity(&CHASSIS.joint_motor[1], DEBUG_VEL_KP);
                delay_us(200);
                DmMitCtrlVelocity(&CHASSIS.joint_motor[2], DEBUG_VEL_KP);
                DmMitCtrlVelocity(&CHASSIS.joint_motor[3], DEBUG_VEL_KP);
            } break;
            case CHASSIS_ZERO_FORCE:
            default: {
                DmMitCtrlVelocity(&CHASSIS.joint_motor[0], ZERO_FORCE_VEL_KP);
                DmMitCtrlVelocity(&CHASSIS.joint_motor[1], ZERO_FORCE_VEL_KP);
                delay_us(200);
                DmMitCtrlVelocity(&CHASSIS.joint_motor[2], ZERO_FORCE_VEL_KP);
                DmMitCtrlVelocity(&CHASSIS.joint_motor[3], ZERO_FORCE_VEL_KP);
            }
        }
    }
}

/**
 * @brief 发送驱动轮电机控制指令
 * @param chassis
 */
static void SendWheelMotorCmd(void)
{
    switch (CHASSIS.mode) {
        case CHASSIS_FOLLOW_GIMBAL_YAW:
        case CHASSIS_STOP:
        case CHASSIS_SPIN:
        case CHASSIS_FREE: {
            LkMultipleTorqueControl(WHEEL_CAN, 100, 100, 0, 0);

        } break;
        case CHASSIS_CALIBRATE: {
            LkMultipleTorqueControl(WHEEL_CAN, 0, 0, 0, 0);
        } break;
        case CHASSIS_DEBUG: {
            LkMultipleTorqueControl(
                WHEEL_CAN, CHASSIS.wheel_motor[0].set.tor, CHASSIS.wheel_motor[1].set.tor, 0, 0);
        } break;
        case CHASSIS_OFF:
        case CHASSIS_ZERO_FORCE:
        default: {
            LkMultipleTorqueControl(WHEEL_CAN, 0, 0, 0, 0);
        }
    }
}

#endif /* CHASSIS_BALANCE */
