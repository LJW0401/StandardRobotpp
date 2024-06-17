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

static GroundTouch_s GROUND_TOUCH = {
    .touch_time = 0,
    .touch = false,
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
            .Tp = TP_RATIO,
            .T = T_RATIO,
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
    CHASSIS.imu = Subscribe("imu_data");      // 获取IMU数据指针

    /*-------------------- 初始化底盘电机 --------------------*/
    MotorInit(&CHASSIS.joint_motor[0], 1, JOINT_CAN, DM_8009, J0_DIRECTION, 1, DM_MODE_MIT);
    MotorInit(&CHASSIS.joint_motor[1], 2, JOINT_CAN, DM_8009, J1_DIRECTION, 1, DM_MODE_MIT);
    MotorInit(&CHASSIS.joint_motor[2], 3, JOINT_CAN, DM_8009, J2_DIRECTION, 1, DM_MODE_MIT);
    MotorInit(&CHASSIS.joint_motor[3], 4, JOINT_CAN, DM_8009, J3_DIRECTION, 1, DM_MODE_MIT);

    MotorInit(&CHASSIS.wheel_motor[0], 1, WHEEL_CAN, MF_9025, W0_DIRECTION, 1, 0);
    MotorInit(&CHASSIS.wheel_motor[1], 2, WHEEL_CAN, MF_9025, W1_DIRECTION, 1, 0);

    /*-------------------- 值归零 --------------------*/
    memset(&CHASSIS.fdb, 0, sizeof(CHASSIS.fdb));
    memset(&CHASSIS.ref, 0, sizeof(CHASSIS.ref));

    /*-------------------- 初始化底盘PID --------------------*/
    // yaw轴跟踪pid
    float yaw_angle_pid[3] = {KP_CHASSIS_YAW_ANGLE, KI_CHASSIS_YAW_ANGLE, KD_CHASSIS_YAW_ANGLE};
    float yaw_velocity_pid[3] = {
        KP_CHASSIS_YAW_VELOCITY, KI_CHASSIS_YAW_VELOCITY, KD_CHASSIS_YAW_VELOCITY};
    PID_init(
        &CHASSIS.pid.yaw_angle, PID_POSITION, yaw_angle_pid, MAX_OUT_CHASSIS_YAW_ANGLE,
        MAX_IOUT_CHASSIS_YAW_ANGLE);
    PID_init(
        &CHASSIS.pid.yaw_velocity, PID_POSITION, yaw_velocity_pid, MAX_OUT_CHASSIS_YAW_VELOCITY,
        MAX_IOUT_CHASSIS_YAW_VELOCITY);

    // 速度增量pid
    float vel_add_pid[3] = {KP_CHASSIS_VEL_ADD, KI_CHASSIS_VEL_ADD, KD_CHASSIS_VEL_ADD};
    PID_init(
        &CHASSIS.pid.vel_add, PID_POSITION, vel_add_pid, MAX_OUT_CHASSIS_VEL_ADD,
        MAX_IOUT_CHASSIS_VEL_ADD);

#if LOCATION_CONTROL

    float roll_angle_pid[3] = {KP_CHASSIS_ROLL_ANGLE, KI_CHASSIS_ROLL_ANGLE, KD_CHASSIS_ROLL_ANGLE};
    float pitch_angle_pid[3] = {
        KP_CHASSIS_PITCH_ANGLE, KI_CHASSIS_PITCH_ANGLE, KD_CHASSIS_PITCH_ANGLE};
    float pitch_vel_pid[3] = {
        KP_CHASSIS_PITCH_VELOCITY, KI_CHASSIS_PITCH_VELOCITY, KD_CHASSIS_PITCH_VELOCITY};

    PID_init(
        &CHASSIS.pid.roll_angle, PID_POSITION, roll_angle_pid, MAX_OUT_CHASSIS_ROLL_ANGLE,
        MAX_IOUT_CHASSIS_ROLL_ANGLE);

    PID_init(
        &CHASSIS.pid.pitch_angle, PID_POSITION, pitch_angle_pid, MAX_OUT_CHASSIS_PITCH_ANGLE,
        MAX_IOUT_CHASSIS_PITCH_ANGLE);
    PID_init(
        &CHASSIS.pid.pitch_vel, PID_POSITION, pitch_vel_pid, MAX_OUT_CHASSIS_PITCH_VELOCITY,
        MAX_IOUT_CHASSIS_PITCH_VELOCITY);
#else
    float roll_angle_pid[3] = {KP_CHASSIS_ROLL_ANGLE, KI_CHASSIS_ROLL_ANGLE, KD_CHASSIS_ROLL_ANGLE};
    // float roll_velocity_pid[3] = {
    //     KP_CHASSIS_ROLL_VELOCITY, KI_CHASSIS_ROLL_VELOCITY, KD_CHASSIS_ROLL_VELOCITY};

    float leg_length_length_pid[3] = {
        KP_CHASSIS_LEG_LENGTH_LENGTH, KI_CHASSIS_LEG_LENGTH_LENGTH, KD_CHASSIS_LEG_LENGTH_LENGTH};
    // float leg_length_speed_pid[3] = {
    //     KP_CHASSIS_LEG_LENGTH_SPEED, KI_CHASSIS_LEG_LENGTH_SPEED, KD_CHASSIS_LEG_LENGTH_SPEED};

    float leg_angle_angle_pid[3] = {
        KP_CHASSIS_LEG_ANGLE_ANGLE, KI_CHASSIS_LEG_ANGLE_ANGLE, KD_CHASSIS_LEG_ANGLE_ANGLE};

    PID_init(
        &CHASSIS.pid.roll_angle, PID_POSITION, roll_angle_pid, MAX_OUT_CHASSIS_ROLL_ANGLE,
        MAX_IOUT_CHASSIS_ROLL_ANGLE);
    // PID_init(
    //     &CHASSIS.pid.roll_velocity, PID_POSITION, roll_velocity_pid, MAX_OUT_CHASSIS_ROLL_VELOCITY,
    //     MAX_IOUT_CHASSIS_ROLL_VELOCITY);

    PID_init(
        &CHASSIS.pid.leg_length_length[0], PID_POSITION, leg_length_length_pid,
        MAX_OUT_CHASSIS_LEG_LENGTH_LENGTH, MAX_IOUT_CHASSIS_LEG_LENGTH_LENGTH);
    // PID_init(
    //     &CHASSIS.pid.leg_length_left_speed, PID_POSITION, leg_length_speed_pid,
    //     MAX_OUT_CHASSIS_LEG_LENGTH_SPEED, MAX_IOUT_CHASSIS_LEG_LENGTH_SPEED);

    PID_init(
        &CHASSIS.pid.leg_length_length[1], PID_POSITION, leg_length_length_pid,
        MAX_OUT_CHASSIS_LEG_LENGTH_LENGTH, MAX_IOUT_CHASSIS_LEG_LENGTH_LENGTH);
    // PID_init(
    //     &CHASSIS.pid.leg_length_right_speed, PID_POSITION, leg_length_speed_pid,
    //     MAX_OUT_CHASSIS_LEG_LENGTH_SPEED, MAX_IOUT_CHASSIS_LEG_LENGTH_SPEED);

    PID_init(
        &CHASSIS.pid.leg_angle_angle, PID_POSITION, leg_angle_angle_pid,
        MAX_OUT_CHASSIS_LEG_ANGLE_ANGLE, MAX_IOUT_CHASSIS_LEG_ANGLE_ANGLE);
#endif

    float stand_up_pid[3] = {KP_CHASSIS_STAND_UP, KI_CHASSIS_STAND_UP, KD_CHASSIS_STAND_UP};
    PID_init(
        &CHASSIS.pid.stand_up, PID_POSITION, stand_up_pid, MAX_OUT_CHASSIS_STAND_UP,
        MAX_IOUT_CHASSIS_STAND_UP);

    float wheel_stop_pid[3] = {KP_CHASSIS_WHEEL_STOP, KI_CHASSIS_WHEEL_STOP, KD_CHASSIS_WHEEL_STOP};
    PID_init(
        &CHASSIS.pid.wheel_stop[0], PID_POSITION, wheel_stop_pid, MAX_OUT_CHASSIS_WHEEL_STOP,
        MAX_IOUT_CHASSIS_WHEEL_STOP);
    PID_init(
        &CHASSIS.pid.wheel_stop[1], PID_POSITION, wheel_stop_pid, MAX_OUT_CHASSIS_WHEEL_STOP,
        MAX_IOUT_CHASSIS_WHEEL_STOP);

    // 初始化低通滤波器
    LowPassFilterInit(&CHASSIS.lpf.leg_length_accel_filter[0], LEG_DDLENGTH_LPF_ALPHA);
    LowPassFilterInit(&CHASSIS.lpf.leg_length_accel_filter[1], LEG_DDLENGTH_LPF_ALPHA);

    LowPassFilterInit(&CHASSIS.lpf.leg_angle_accel_filter[0], LEG_DDANGLE_LPF_ALPHA);
    LowPassFilterInit(&CHASSIS.lpf.leg_angle_accel_filter[1], LEG_DDANGLE_LPF_ALPHA);

    LowPassFilterInit(&CHASSIS.lpf.support_force_filter[0], LEG_SUPPORT_FORCE_LPF_ALPHA);
    LowPassFilterInit(&CHASSIS.lpf.support_force_filter[1], LEG_SUPPORT_FORCE_LPF_ALPHA);
}

/*-------------------- Handle exception --------------------*/

static void GroundTouchDectect(void);

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

    if (CHASSIS.imu == NULL) {
        CHASSIS.error_code |= IMU_ERROR_OFFSET;
    } else {
        CHASSIS.error_code &= ~IMU_ERROR_OFFSET;
    }

    for (uint8_t i = 0; i < 4; i++) {
        if (fabs(CHASSIS.joint_motor[i].fdb.tor) > MAX_TORQUE_PROTECT) {
            CHASSIS.error_code |= JOINT_ERROR_OFFSET;
            break;
        }
    }

    if ((CHASSIS.mode == CHASSIS_OFF || CHASSIS.mode == CHASSIS_SAFE) &&
        fabs(CHASSIS.pid.stand_up.max_out) != 0.0f) {
        PID_clear(&CHASSIS.pid.stand_up);
    }

    GroundTouchDectect();  // 离地检测
}

/**
 * @brief 离地检测
 * @param  
 */
static void GroundTouchDectect(void)
{
    for (uint8_t i = 0; i < 2; i++) {
        float Theta = CHASSIS.fdb.leg[i].rod.Angle - M_PI_2 - CHASSIS.imu->pitch;
        float dTheta = CHASSIS.fdb.leg[i].rod.dAngle - CHASSIS.imu->pitch_vel;
        float ddTheta = CHASSIS.fdb.leg[i].rod.ddAngle;

        float L0 = CHASSIS.fdb.leg[i].rod.Length;
        float dL0 = CHASSIS.fdb.leg[i].rod.dLength;
        float ddL0 = CHASSIS.fdb.leg[i].rod.ddLength;

        float ddz_M = CHASSIS.imu->z_accel - GRAVITY;
        float ddz_w = ddz_M - ddL0 * cosf(Theta) + 2 * dL0 * dTheta * sinf(Theta) +
                      L0 * ddTheta * sinf(Theta) + L0 * dTheta * dTheta * cosf(Theta);

        float F = CHASSIS.ref.leg[i].rod.F;
        float Tp = CHASSIS.ref.leg[i].rod.Tp;
        float P = F * cosf(Theta) + Tp * sinf(Theta) / L0;

        float Fn = P + WHEEL_MASS * GRAVITY + WHEEL_MASS * ddz_w;
        GROUND_TOUCH.support_force[i] = LowPassFilterCalc(&CHASSIS.lpf.support_force_filter[i], Fn);

        // if (i == 1) {
        //     OutputPCData.packets[15].data = P;
        //     OutputPCData.packets[16].data = WHEEL_MASS * GRAVITY;
        //     OutputPCData.packets[17].data = ddz_w;
        //     OutputPCData.packets[18].data = Theta;
        //     OutputPCData.packets[19].data = F;
        //     OutputPCData.packets[20].data = Tp;
        // }
    }

    // GROUND_TOUCH.support_force[0] =
    //     CHASSIS.ref.leg[0].rod.F +
    //     LEG_MASS * (GRAVITY - (CHASSIS.fdb.leg[0].rod.ddLength - CHASSIS.imu->z_accel));

    // bool touch = false;

    uint32_t now = HAL_GetTick();
    if (now - GROUND_TOUCH.touch_time < MAX_TOUCH_INTERVAL) {
        //若上次触地时间距离现在不超过 MAX_TOUCH_INTERVAL ms，则认为当前瞬间接地，避免弹跳导致误判
        GROUND_TOUCH.touch = true;
    } else {
        GROUND_TOUCH.touch = false;
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
    static ChassisMode_e last_mode;
    last_mode = CHASSIS.mode;

    if (CHASSIS.error_code & DBUS_ERROR_OFFSET) {  // 遥控器出错时的状态处理
        CHASSIS.mode = CHASSIS_SAFE;
        return;
    }

    if (CHASSIS.error_code & IMU_ERROR_OFFSET) {  // IMU出错时的状态处理
        CHASSIS.mode = CHASSIS_SAFE;
        return;
    }

    if (CHASSIS.error_code & JOINT_ERROR_OFFSET) {  // 关节电机出错时的状态处理
        CHASSIS.mode = CHASSIS_SAFE;
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
        CHASSIS.mode = CHASSIS_CUSTOM;
    } else if (switch_is_down(CHASSIS.rc->rc.s[CHASSIS_MODE_CHANNEL])) {
        CHASSIS.mode = CHASSIS_SAFE;
    }

    switch (CHASSIS.mode)  //进入起立模式的判断
    {
        case CHASSIS_FREE:
        case CHASSIS_FOLLOW_GIMBAL_YAW:
        case CHASSIS_CUSTOM: {  // 若上一次模式为底盘关闭或底盘安全，则进入起立模式
            if (last_mode == CHASSIS_OFF ||       // 上一次模式为底盘关闭
                last_mode == CHASSIS_SAFE ||      // 上一次模式为底盘安全
                last_mode == CHASSIS_STAND_UP) {  // 上一次模式为起立模式
                CHASSIS.mode = CHASSIS_STAND_UP;
            }
        } break;
        default:
            break;
    }

    if (CHASSIS.mode == CHASSIS_STAND_UP && fabs(CHASSIS.fdb.phi) < 0.1f) {
        CHASSIS.mode = CHASSIS_CUSTOM;
    }
}

/*-------------------- Observe --------------------*/

#define ZERO_POS_THRESHOLD 0.001f

static void UpdateLegStatus(void);
static void UpdateMotorStatus(void);

/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void ChassisObserver(void)
{
    UpdateMotorStatus();
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
    CHASSIS.fdb.theta     = (CHASSIS.fdb.leg[0].rod.Angle + CHASSIS.fdb.leg[1].rod.Angle) / 2 
                            - M_PI_2 - CHASSIS.imu->pitch;
    CHASSIS.fdb.theta_dot = (CHASSIS.fdb.leg[0].rod.dAngle + CHASSIS.fdb.leg[1].rod.dAngle) / 2 
                            - CHASSIS.imu->pitch_vel;
    CHASSIS.fdb.x         = (CHASSIS.fdb.leg[0].wheel.Angle + CHASSIS.fdb.leg[1].wheel.Angle) / 2;
    CHASSIS.fdb.x_dot     = WHEEL_RADIUS * (CHASSIS.fdb.leg[0].wheel.Velocity + CHASSIS.fdb.leg[1].wheel.Velocity) / 2;
    CHASSIS.fdb.phi       = CHASSIS.imu->pitch;
    CHASSIS.fdb.phi_dot   = CHASSIS.imu->pitch_vel;
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

    // OutputPCData.packets[0].data = CHASSIS.fdb.leg[0].rod.Length;
    // OutputPCData.packets[1].data = CHASSIS.ref.leg[0].rod.Length;
    // OutputPCData.packets[2].data = CHASSIS.fdb.leg[1].rod.Length;
    // OutputPCData.packets[3].data = CHASSIS.ref.leg[1].rod.Length;
    OutputPCData.packets[4].data = CHASSIS.joint_motor[0].fdb.tor;
    OutputPCData.packets[5].data = CHASSIS.joint_motor[1].fdb.tor;
    OutputPCData.packets[6].data = CHASSIS.joint_motor[2].fdb.tor;
    OutputPCData.packets[7].data = CHASSIS.joint_motor[3].fdb.tor;
    // OutputPCData.packets[8].data = CHASSIS.pid.leg_length_length[0].out;
    OutputPCData.packets[9].data = CHASSIS.joint_motor[0].set.tor;
    OutputPCData.packets[10].data = CHASSIS.joint_motor[1].set.tor;
    OutputPCData.packets[11].data = CHASSIS.joint_motor[2].set.tor;
    OutputPCData.packets[12].data = CHASSIS.joint_motor[3].set.tor;
    OutputPCData.packets[13].data = GROUND_TOUCH.support_force[0];
    OutputPCData.packets[14].data = GROUND_TOUCH.support_force[1];
    OutputPCData.packets[15].data = CHASSIS.imu->roll;
    OutputPCData.packets[16].data = CHASSIS.wheel_motor[0].set.tor;
    OutputPCData.packets[17].data = CHASSIS.wheel_motor[1].set.tor;
    // OutputPCData.packets[18].data = CHASSIS.wheel_motor[0].set.tor;
    // OutputPCData.packets[19].data = CHASSIS.wheel_motor[1].set.tor;
    // OutputPCData.packets[20].data = CHASSIS.imu->z_accel;
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
    // =====更新关节姿态=====
    CHASSIS.fdb.leg[0].joint[0].Angle =
        theta_transform(CHASSIS.joint_motor[0].fdb.pos, J0_ANGLE_OFFSET, J0_DIRECTION, 1);
    CHASSIS.fdb.leg[0].joint[1].Angle =
        theta_transform(CHASSIS.joint_motor[1].fdb.pos, J1_ANGLE_OFFSET, J1_DIRECTION, 1);
    CHASSIS.fdb.leg[1].joint[0].Angle =
        theta_transform(CHASSIS.joint_motor[2].fdb.pos, J2_ANGLE_OFFSET, J2_DIRECTION, 1);
    CHASSIS.fdb.leg[1].joint[1].Angle =
        theta_transform(CHASSIS.joint_motor[3].fdb.pos, J3_ANGLE_OFFSET, J3_DIRECTION, 1);

    CHASSIS.fdb.leg[0].joint[0].dAngle = CHASSIS.joint_motor[0].fdb.vel;
    CHASSIS.fdb.leg[0].joint[1].dAngle = CHASSIS.joint_motor[1].fdb.vel;
    CHASSIS.fdb.leg[1].joint[0].dAngle = CHASSIS.joint_motor[2].fdb.vel;
    CHASSIS.fdb.leg[1].joint[1].dAngle = CHASSIS.joint_motor[3].fdb.vel;

    // =====更新驱动轮姿态=====
    CHASSIS.fdb.leg[0].wheel.Velocity = CHASSIS.wheel_motor[0].fdb.vel * (W0_DIRECTION);
    CHASSIS.fdb.leg[1].wheel.Velocity = CHASSIS.wheel_motor[1].fdb.vel * (W1_DIRECTION);

    // =====更新摆杆姿态=====
    double leg_pos[2];
    double leg_speed[2];

    float last_dLength, last_dAngle;

    for (uint8_t i = 0; i < 2; i++) {
        // 更新位置信息
        LegFKine(CHASSIS.fdb.leg[i].joint[1].Angle, CHASSIS.fdb.leg[i].joint[0].Angle, leg_pos);
        CHASSIS.fdb.leg[i].rod.Length = leg_pos[0];
        CHASSIS.fdb.leg[i].rod.Angle = leg_pos[1];

        // 更新速度信息
        // clang-format off
        last_dLength = CHASSIS.fdb.leg[i].rod.dLength;
        last_dAngle = CHASSIS.fdb.leg[i].rod.dAngle;
        LegSpeed(
            CHASSIS.fdb.leg[i].joint[1].dAngle, CHASSIS.fdb.leg[i].joint[0].dAngle,
            CHASSIS.fdb.leg[i].joint[1].Angle , CHASSIS.fdb.leg[i].joint[0].Angle,
            leg_speed);
        CHASSIS.fdb.leg[i].rod.dLength = leg_speed[0];
        CHASSIS.fdb.leg[i].rod.dAngle  = leg_speed[1];
        // clang-format on

        // 更新加速度信息
        float accel = (CHASSIS.fdb.leg[i].rod.dLength - last_dLength) / CHASSIS_CONTROL_TIME_S;
        CHASSIS.fdb.leg[i].rod.ddLength =
            LowPassFilterCalc(&CHASSIS.lpf.leg_length_accel_filter[i], accel);

        accel = (CHASSIS.fdb.leg[i].rod.dAngle - last_dAngle) / CHASSIS_CONTROL_TIME_S;
        CHASSIS.fdb.leg[i].rod.ddAngle =
            LowPassFilterCalc(&CHASSIS.lpf.leg_angle_accel_filter[i], accel);
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
    int16_t rc_x = 0, rc_wz = 0;
    int16_t rc_length = 0, rc_roll = 0;
    rc_deadband_limit(CHASSIS.rc->rc.ch[CHASSIS_X_CHANNEL], rc_x, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(CHASSIS.rc->rc.ch[CHASSIS_WZ_CHANNEL], rc_wz, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(CHASSIS.rc->rc.ch[CHASSIS_LENGTH_CHANNEL], rc_length, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(CHASSIS.rc->rc.ch[CHASSIS_ROLL_CHANNEL], rc_roll, CHASSIS_RC_DEADLINE);

    ChassisSpeedVector_t v_set = {0.0f, 0.0f, 0.0f};

    v_set.vx = rc_x * RC_TO_ONE * MAX_SPEED_VECTOR_VX;
    v_set.vy = 0;
    v_set.wz = -rc_wz * RC_TO_ONE * MAX_SPEED_VECTOR_WZ;
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
        case CHASSIS_STAND_UP: {
            v_set.vx = 0;
            v_set.vy = 0;
            v_set.wz = 0;
        } break;
        default:
            break;
    }

    float v = v_set.vx;
    float x;

    if (fabs(v) > WHEEL_DEADZONE) {  // 运动状态，只需控制速度
        x = CHASSIS.fdb.x;
    } else {
        if (CHASSIS.ref.speed_vector.vx > WHEEL_DEADZONE) {
            // 进入停止状态，需要加入位置控制
            x = (CHASSIS.fdb.leg[0].wheel.Angle + CHASSIS.fdb.leg[1].wheel.Angle) / 2;
        } else {  // 还是停止状态，保留原位置
            x = CHASSIS.ref.x;
        }
    }

    CHASSIS.ref.speed_vector.vx = v_set.vx;
    CHASSIS.ref.speed_vector.vy = 0;
    CHASSIS.ref.speed_vector.wz = v_set.wz;

    // clang-format off
    CHASSIS.ref.theta     = 0;
    CHASSIS.ref.theta_dot = 0;
    CHASSIS.ref.x         = x;
    CHASSIS.ref.x_dot     = fp32_constrain(CHASSIS.ref.speed_vector.vx, MIN_SPEED, MAX_SPEED);
    CHASSIS.ref.phi       = 0;
    CHASSIS.ref.phi_dot   = 0;
    // clang-format on

    static float vel_add = 0;  // 速度增量，用于适应重心位置变化
    if (fabs(CHASSIS.ref.x_dot) < WHEEL_DEADZONE && fabs(CHASSIS.fdb.x_dot) < 0.8f) {
        // 当目标速度为0，且速度小于阈值时，增加速度增量
        vel_add = PID_calc(&CHASSIS.pid.vel_add, CHASSIS.fdb.x_dot, CHASSIS.ref.x_dot);
    }
    CHASSIS.ref.x_dot += vel_add;
    CHASSIS.ref.x_dot = fp32_constrain(
        CHASSIS.ref.x_dot,                              //ref
        CHASSIS.fdb.x_dot + MIN_DELTA_VEL_FDB_TO_REF,   //min
        CHASSIS.fdb.x_dot + MAX_DELTA_VEL_FDB_TO_REF);  //max

    static float angle = M_PI_2;
    static float length = 0.25f;
    switch (CHASSIS.mode) {
        case CHASSIS_STAND_UP: {
            length = 0.12f;
            angle = M_PI_2;
        } break;
        case CHASSIS_CUSTOM:
        case CHASSIS_DEBUG: {
            angle = M_PI_2;  // + rc_angle * RC_TO_ONE * 0.3f;
            length += rc_length * RC_LENGTH_ADD_RATIO;
        } break;
        case CHASSIS_FREE: {
        } break;
        case CHASSIS_FOLLOW_GIMBAL_YAW:
        default: {
            angle = M_PI_2;
            length = 0.25f;
        }
    }
    length = fp32_constrain(length, MIN_LEG_LENGTH, MAX_LEG_LENGTH);
    angle = fp32_constrain(angle, MIN_LEG_ANGLE, MAX_LEG_ANGLE);

    CHASSIS.ref.leg[0].rod.Length = length;
    CHASSIS.ref.leg[1].rod.Length = length;

    CHASSIS.ref.leg[0].rod.Angle = angle;
    CHASSIS.ref.leg[1].rod.Angle = angle;

    CHASSIS.ref.roll = fp32_constrain(rc_roll * RC_TO_ONE * MAX_ROLL, MIN_ROLL, MAX_ROLL);
}

/*-------------------- Console --------------------*/

static void LocomotionController(float Tp[2], float T_w[2]);
#if LOCATION_CONTROL
static void LegController(double joint_pos_l[2], double joint_pos_r[2]);
#else
static float LegFeedForward(float theta);
static void LegController(float F[2]);
#endif
static void SetK(float leg_length, float k[2][6]);
static void LQRFeedbackCalc(float k[2][6], float x[6], float t[2]);

static void ConsoleZeroForce(void);
static void ConsoleCalibrate(void);
static void ConsoleNormal(void);
static void ConsoleDebug(void);
static void ConsoleStandUp(void);

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
        case CHASSIS_CUSTOM:
        case CHASSIS_FREE: {
            ConsoleNormal();
        } break;
        case CHASSIS_DEBUG: {
            ConsoleDebug();
        } break;
        case CHASSIS_STAND_UP: {
            ConsoleStandUp();
        } break;
        case CHASSIS_OFF:
        case CHASSIS_SAFE:
        default: {
            ConsoleZeroForce();
        }
    }
}

/**
 * @brief      运动控制器
 * @param[out]  Tp 输出的髋关节力矩 0-左，1-右
 * @param[out]  T_w 输出的驱动轮力矩  0-左，1-右
 */
static void LocomotionController(float Tp[2], float T_w[2])
{
    float x[6];
    // clang-format off
    x[0] = CHASSIS.fdb.theta     - CHASSIS.ref.theta;
    x[1] = CHASSIS.fdb.theta_dot - CHASSIS.ref.theta_dot;
    x[2] = CHASSIS.fdb.x         - CHASSIS.ref.x;
    x[3] = CHASSIS.fdb.x_dot     - CHASSIS.ref.x_dot;
    x[4] = CHASSIS.fdb.phi       - CHASSIS.ref.phi;
    x[5] = CHASSIS.fdb.phi_dot   - CHASSIS.ref.phi_dot;
    // clang-format on

    float leg_length = (CHASSIS.fdb.leg[0].rod.Length + CHASSIS.fdb.leg[1].rod.Length) / 2;
    float k[2][6];
    SetK(leg_length, k);
    float t_tp[2];
    LQRFeedbackCalc(k, x, t_tp);
    float t = t_tp[0] * CHASSIS.ratio.T;
    float tp = t_tp[1] * CHASSIS.ratio.Tp;

    float dyaw;
    dyaw = CHASSIS.ref.yaw - CHASSIS.fdb.yaw;
    if (dyaw > M_PI) {
        dyaw -= 2 * M_PI;
    } else if (dyaw < -M_PI) {
        dyaw += 2 * M_PI;
    }

    if (CHASSIS.mode == CHASSIS_FOLLOW_GIMBAL_YAW) {
        //这里输入的dyaw是需要取反的原因还不是很清楚
        PID_calc(&CHASSIS.pid.yaw_angle, -dyaw, 0);
        PID_calc(&CHASSIS.pid.yaw_velocity, CHASSIS.fdb.yaw_velocity, CHASSIS.pid.yaw_angle.out);
    } else {
        PID_calc(&CHASSIS.pid.yaw_velocity, CHASSIS.fdb.yaw_velocity, CHASSIS.ref.speed_vector.wz);
    }

#if LOCATION_CONTROL
    T_w[0] = t + CHASSIS.pid.yaw_velocity.out;
    T_w[1] = t - CHASSIS.pid.yaw_velocity.out;
    Tp[0] = 0;
    Tp[1] = 0;
#else
    float dangle = CHASSIS.fdb.leg[0].rod.Angle - CHASSIS.fdb.leg[1].rod.Angle;
    PID_calc(&CHASSIS.pid.leg_angle_angle, dangle, 0);

    T_w[0] = t + CHASSIS.pid.yaw_velocity.out;
    T_w[1] = t - CHASSIS.pid.yaw_velocity.out;
    Tp[0] = tp + CHASSIS.pid.leg_angle_angle.out;
    Tp[1] = tp - CHASSIS.pid.leg_angle_angle.out;
#endif
}

#if !LOCATION_CONTROL
/**
 * @brief        前馈控制
 * @param[in]    theta 当前腿与竖直方向夹角
 * @return       前馈量
 */
static float LegFeedForward(float theta) { return BODY_MASS * GRAVITY * cosf(theta) / 2; }
#endif

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
#if LOCATION_CONTROL
/**
 * @brief 腿部控制器
 * @param[out]  joint_pos_l 左关节电机设定位置 0-后，1-前
 * @param[out]  joint_pos_r 右关节电机设定位置 0-后，1-前
 */
static void LegController(double joint_pos_l[2], double joint_pos_r[2])
{
    float dAngle = PID_calc(&CHASSIS.pid.pitch_angle, CHASSIS.fdb.phi, CHASSIS.ref.phi);
    float delta_Angle = PID_calc(&CHASSIS.pid.pitch_angle, CHASSIS.fdb.phi_dot, dAngle);
    delta_Angle = 0;

    CHASSIS.ref.leg[0].rod.Angle = M_PI_2 + delta_Angle;
    CHASSIS.ref.leg[1].rod.Angle = M_PI_2 + delta_Angle;

    CHASSIS.ref.leg[0].rod.Angle =
        fp32_constrain(CHASSIS.ref.leg[0].rod.Angle, MIN_LEG_ANGLE, MAX_LEG_ANGLE);
    CHASSIS.ref.leg[1].rod.Angle =
        fp32_constrain(CHASSIS.ref.leg[1].rod.Angle, MIN_LEG_ANGLE, MAX_LEG_ANGLE);

    float delta_Length =
        PID_calc(&CHASSIS.pid.roll_angle, CHASSIS.fdb.roll, CHASSIS.ref.roll);  // 腿长补偿

    CHASSIS.ref.leg[0].rod.Length += delta_Length * DLENGTH_DIRECTION;
    CHASSIS.ref.leg[1].rod.Length -= delta_Length * DLENGTH_DIRECTION;

    CHASSIS.ref.leg[0].rod.Length =
        fp32_constrain(CHASSIS.ref.leg[0].rod.Length, MIN_LEG_LENGTH, MAX_LEG_LENGTH);
    CHASSIS.ref.leg[1].rod.Length =
        fp32_constrain(CHASSIS.ref.leg[1].rod.Length, MIN_LEG_LENGTH, MAX_LEG_LENGTH);

    LegIKine(CHASSIS.ref.leg[0].rod.Length, CHASSIS.ref.leg[0].rod.Angle, joint_pos_l);
    LegIKine(CHASSIS.ref.leg[1].rod.Length, CHASSIS.ref.leg[1].rod.Angle, joint_pos_r);
}
#else
/**
 * @brief 腿部控制器
 * @param[out]  F 输出的腿部沿杆方向的力F 0-左，1-右
 */
static void LegController(float F[2])
{
    PID_calc(
        &CHASSIS.pid.leg_length_length[0], CHASSIS.fdb.leg[0].rod.Length,
        CHASSIS.ref.leg[0].rod.Length);
    float theta_l = CHASSIS.fdb.leg[0].rod.Angle - M_PI_2 - CHASSIS.imu->pitch;
    float fdf_left = LegFeedForward(theta_l) * FF_RATIO;

    PID_calc(
        &CHASSIS.pid.leg_length_length[1], CHASSIS.fdb.leg[1].rod.Length,
        CHASSIS.ref.leg[1].rod.Length);
    float theta_r = CHASSIS.fdb.leg[1].rod.Angle - M_PI_2 - CHASSIS.imu->pitch;
    float fdf_right = LegFeedForward(theta_r) * FF_RATIO;

    PID_calc(&CHASSIS.pid.roll_angle, CHASSIS.fdb.roll, CHASSIS.ref.roll);
    // PID_calc(&CHASSIS.pid.roll_velocity, CHASSIS.fdb.roll_velocity, CHASSIS.pid.roll_angle.out);

    F[0] = CHASSIS.pid.leg_length_length[0].out + fdf_left + CHASSIS.pid.roll_angle.out;
    F[1] = CHASSIS.pid.leg_length_length[1].out + fdf_right - CHASSIS.pid.roll_angle.out;
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

    CHASSIS.wheel_motor[0].set.vel = 0;
    CHASSIS.wheel_motor[1].set.vel = 0;

    CHASSIS.wheel_motor[0].set.value =
        PID_calc(&CHASSIS.pid.wheel_stop[0], CHASSIS.wheel_motor[0].fdb.vel, 0);
    CHASSIS.wheel_motor[1].set.value =
        PID_calc(&CHASSIS.pid.wheel_stop[1], CHASSIS.wheel_motor[1].fdb.vel, 0);
}

static void ConsoleCalibrate(void)
{
    CHASSIS.joint_motor[0].set.vel = -CALIBRATE_VELOCITY;
    CHASSIS.joint_motor[1].set.vel = CALIBRATE_VELOCITY;
    CHASSIS.joint_motor[2].set.vel = CALIBRATE_VELOCITY;
    CHASSIS.joint_motor[3].set.vel = -CALIBRATE_VELOCITY;

    CHASSIS.wheel_motor[0].set.tor = 0;
    CHASSIS.wheel_motor[1].set.tor = 0;
}

static void ConsoleNormal(void)
{
    float tp[2], t[2];
    LocomotionController(tp, t);
    CHASSIS.ref.leg[0].rod.Tp = tp[0];
    CHASSIS.ref.leg[1].rod.Tp = tp[1];

#if LOCATION_CONTROL
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
    }
    // 检测设定角度是否超过电机角度限制
    CHASSIS.joint_motor[0].set.pos =
        fp32_constrain(CHASSIS.joint_motor[0].set.pos, MIN_J0_ANGLE, MAX_J0_ANGLE);
    CHASSIS.joint_motor[1].set.pos =
        fp32_constrain(CHASSIS.joint_motor[1].set.pos, MIN_J1_ANGLE, MAX_J1_ANGLE);
    CHASSIS.joint_motor[2].set.pos =
        fp32_constrain(CHASSIS.joint_motor[2].set.pos, MIN_J2_ANGLE, MAX_J2_ANGLE);
    CHASSIS.joint_motor[3].set.pos =
        fp32_constrain(CHASSIS.joint_motor[3].set.pos, MIN_J3_ANGLE, MAX_J3_ANGLE);
#else
    float F[2];
    LegController(F);
    CHASSIS.ref.leg[0].rod.F = F[0];
    CHASSIS.ref.leg[1].rod.F = F[1];

    double joint_torque[2];
    LegTransform(
        CHASSIS.ref.leg[0].rod.F, CHASSIS.ref.leg[0].rod.Tp, CHASSIS.fdb.leg[0].joint[1].Angle,
        CHASSIS.fdb.leg[0].joint[0].Angle, joint_torque);

    CHASSIS.joint_motor[0].set.tor = -joint_torque[0] * (J0_DIRECTION);
    CHASSIS.joint_motor[1].set.tor = -joint_torque[1] * (J1_DIRECTION);

    LegTransform(
        CHASSIS.ref.leg[1].rod.F, CHASSIS.ref.leg[1].rod.Tp, CHASSIS.fdb.leg[1].joint[1].Angle,
        CHASSIS.fdb.leg[1].joint[0].Angle, joint_torque);

    CHASSIS.joint_motor[2].set.tor = -joint_torque[0] * (J2_DIRECTION);
    CHASSIS.joint_motor[3].set.tor = -joint_torque[1] * (J3_DIRECTION);

    CHASSIS.joint_motor[0].set.tor =
        fp32_constrain(CHASSIS.joint_motor[0].set.tor, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
    CHASSIS.joint_motor[1].set.tor =
        fp32_constrain(CHASSIS.joint_motor[1].set.tor, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
    CHASSIS.joint_motor[2].set.tor =
        fp32_constrain(CHASSIS.joint_motor[2].set.tor, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
    CHASSIS.joint_motor[3].set.tor =
        fp32_constrain(CHASSIS.joint_motor[3].set.tor, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
#endif

    // QUESTION: 排查电机发送的力矩要反向的问题，这种情况下控制正常
    CHASSIS.wheel_motor[0].set.tor = -(t[0] * (W0_DIRECTION));  //不知道为什么要反向，待后续研究
    CHASSIS.wheel_motor[1].set.tor = -(t[1] * (W1_DIRECTION));  //不知道为什么要反向，待后续研究
}

static void ConsoleDebug(void)
{
    CHASSIS.joint_motor[0].set.tor = 0;
    CHASSIS.joint_motor[1].set.tor = 0;
    CHASSIS.joint_motor[2].set.tor = 0;
    CHASSIS.joint_motor[3].set.tor = 0;

    CHASSIS.joint_motor[0].set.vel = 0;
    CHASSIS.joint_motor[1].set.vel = 0;
    CHASSIS.joint_motor[2].set.vel = 0;
    CHASSIS.joint_motor[3].set.vel = 0;

    CHASSIS.wheel_motor[0].set.value = CHASSIS.wheel_motor[0].set.tor =
        CHASSIS.rc->rc.ch[1] * RC_TO_ONE * 100;
    CHASSIS.wheel_motor[1].set.value = CHASSIS.wheel_motor[1].set.tor =
        CHASSIS.rc->rc.ch[1] * RC_TO_ONE * 100;
}

static void ConsoleStandUp(void)
{
    // ===腿部位置控制===

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
    }
    // 检测设定角度是否超过电机角度限制
    CHASSIS.joint_motor[0].set.pos =
        fp32_constrain(CHASSIS.joint_motor[0].set.pos, MIN_J0_ANGLE, MAX_J0_ANGLE);
    CHASSIS.joint_motor[1].set.pos =
        fp32_constrain(CHASSIS.joint_motor[1].set.pos, MIN_J1_ANGLE, MAX_J1_ANGLE);
    CHASSIS.joint_motor[2].set.pos =
        fp32_constrain(CHASSIS.joint_motor[2].set.pos, MIN_J2_ANGLE, MAX_J2_ANGLE);
    CHASSIS.joint_motor[3].set.pos =
        fp32_constrain(CHASSIS.joint_motor[3].set.pos, MIN_J3_ANGLE, MAX_J3_ANGLE);

    // ===驱动轮pid控制===
    float feedforward = -220;
    PID_calc(&CHASSIS.pid.stand_up, CHASSIS.fdb.phi, 0);
    CHASSIS.wheel_motor[0].set.value = (feedforward + CHASSIS.pid.stand_up.out) * W0_DIRECTION;
    CHASSIS.wheel_motor[1].set.value = (feedforward + CHASSIS.pid.stand_up.out) * W1_DIRECTION;
}

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

        switch (CHASSIS.mode) {
            case CHASSIS_FOLLOW_GIMBAL_YAW:
            case CHASSIS_CUSTOM:
            case CHASSIS_FREE: {
#if LOCATION_CONTROL
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
            case CHASSIS_STAND_UP: {
                DmMitCtrlPosition(&CHASSIS.joint_motor[0], NORMAL_POS_KP, NORMAL_POS_KD);
                DmMitCtrlPosition(&CHASSIS.joint_motor[1], NORMAL_POS_KP, NORMAL_POS_KD);
                delay_us(200);
                DmMitCtrlPosition(&CHASSIS.joint_motor[2], NORMAL_POS_KP, NORMAL_POS_KD);
                DmMitCtrlPosition(&CHASSIS.joint_motor[3], NORMAL_POS_KP, NORMAL_POS_KD);
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
            case CHASSIS_DEBUG:
            case CHASSIS_SAFE:
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
        case CHASSIS_CUSTOM:
        case CHASSIS_FREE: {
            LkMultipleTorqueControl(
                WHEEL_CAN, CHASSIS.wheel_motor[0].set.tor, CHASSIS.wheel_motor[1].set.tor, 0, 0);
        } break;
        case CHASSIS_STAND_UP: {
            LkMultipleIqControl(
                WHEEL_CAN, CHASSIS.wheel_motor[0].set.value, CHASSIS.wheel_motor[1].set.value, 0,
                0);
        } break;
        case CHASSIS_CALIBRATE: {
            LkMultipleTorqueControl(WHEEL_CAN, 0, 0, 0, 0);
        } break;
        case CHASSIS_OFF: {
            LkMultipleTorqueControl(WHEEL_CAN, 0, 0, 0, 0);
        } break;
        case CHASSIS_DEBUG:
        case CHASSIS_SAFE:
        default: {
            LkMultipleTorqueControl(
                WHEEL_CAN, CHASSIS.wheel_motor[0].set.value, CHASSIS.wheel_motor[1].set.value, 0,
                0);
        }
    }
}

#endif /* CHASSIS_BALANCE */
