/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       mechanical_arm.c/h
  * @brief      机械臂功能。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.1     Apr-21-2024     Penguin         1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#include "mechanical_arm_penguin_mini.h"

#if (MECHANICAL_ARM_TYPE == MECHANICAL_ARM_PENGUIN_MINI_ARM)
#include <stdbool.h>

#include "CAN_communication.h"
#include "bsp_delay.h"
#include "custom_controller_connect.h"
#include "detect_task.h"
#include "math.h"
#include "pid.h"
#include "signal_generator.h"
#include "usb_task.h"
#include "user_lib.h"

static MechanicalArm_s MECHANICAL_ARM = {
    .mode = MECHANICAL_ARM_ZERO_FORCE,
    .ctrl_link = LINK_NONE,
    .error_code = 0,
    .zero_setted = false,
    .ref =
        {
            .pos = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        },
    .fdb =
        {
            .pos = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        },
    .upper_limit =
        {
            .pos =
                {MAX_JOINT_0_POSITION, MAX_JOINT_1_POSITION, MAX_JOINT_2_POSITION,
                 MAX_JOINT_3_POSITION, MAX_JOINT_4_POSITION},
        },
    .lower_limit =
        {
            .pos =
                {MIN_JOINT_0_POSITION, MIN_JOINT_1_POSITION, MIN_JOINT_2_POSITION,
                 MIN_JOINT_3_POSITION, MIN_JOINT_4_POSITION},
        },
    .init_completed = {true, false, false, true, true},
};

/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param[in]      none
 * @retval         none
 */
void MechanicalArmInit(void)
{
    // #Motor init ---------------------
    MotorInit(
        &MECHANICAL_ARM.joint_motor[0], 1, 1, CYBERGEAR_MOTOR, JOINT_MOTOR_0_DIRECTION,
        JOINT_MOTOR_0_REDUCTION_RATIO, JOINT_MOTOR_0_MODE);
    MotorInit(
        &MECHANICAL_ARM.joint_motor[1], 2, 1, CYBERGEAR_MOTOR, JOINT_MOTOR_1_DIRECTION,
        JOINT_MOTOR_1_REDUCTION_RATIO, JOINT_MOTOR_1_MODE);
    MotorInit(
        &MECHANICAL_ARM.joint_motor[2], 3, 1, CYBERGEAR_MOTOR, JOINT_MOTOR_2_DIRECTION,
        JOINT_MOTOR_2_REDUCTION_RATIO, JOINT_MOTOR_2_MODE);
    MotorInit(
        &MECHANICAL_ARM.joint_motor[3], JOINT_MOTOR_3_ID, JOINT_MOTOR_3_CAN, JOINT_MOTOR_3_TYPE,
        JOINT_MOTOR_3_DIRECTION, JOINT_MOTOR_3_REDUCTION_RATIO, JOINT_MOTOR_3_MODE);
    MotorInit(
        &MECHANICAL_ARM.joint_motor[4], JOINT_MOTOR_4_ID, JOINT_MOTOR_4_CAN, JOINT_MOTOR_4_TYPE,
        JOINT_MOTOR_4_DIRECTION, JOINT_MOTOR_4_REDUCTION_RATIO, JOINT_MOTOR_4_MODE);

    // #PID init ---------------------
    float pid_joint_3_angle[3] = {KP_JOINT_3_ANGLE, KI_JOINT_3_ANGLE, KD_JOINT_3_ANGLE};
    float pid_joint_3_speed[3] = {KP_JOINT_3_SPEED, KI_JOINT_3_SPEED, KD_JOINT_3_SPEED};
    float pid_joint_4_angle[3] = {KP_JOINT_4_ANGLE, KI_JOINT_4_ANGLE, KD_JOINT_4_ANGLE};
    float pid_joint_4_speed[3] = {KP_JOINT_4_SPEED, KI_JOINT_4_SPEED, KD_JOINT_4_SPEED};
    PID_init(
        &MECHANICAL_ARM.pid.joint_angle[3], PID_POSITION, pid_joint_3_angle, MAX_OUT_JOINT_3_ANGLE,
        MAX_IOUT_JOINT_3_ANGLE);
    PID_init(
        &MECHANICAL_ARM.pid.joint_speed[3], PID_POSITION, pid_joint_3_speed, MAX_OUT_JOINT_3_SPEED,
        MAX_IOUT_JOINT_3_SPEED);
    PID_init(
        &MECHANICAL_ARM.pid.joint_angle[4], PID_POSITION, pid_joint_4_angle, MAX_OUT_JOINT_4_ANGLE,
        MAX_IOUT_JOINT_4_ANGLE);
    PID_init(
        &MECHANICAL_ARM.pid.joint_speed[4], PID_POSITION, pid_joint_4_speed, MAX_OUT_JOINT_4_SPEED,
        MAX_IOUT_JOINT_4_SPEED);

    MECHANICAL_ARM.rc = get_remote_control_point();
}

/*-------------------- Handle exception --------------------*/
void MechanicalArmHandleException(void)
{
    // DBUS error handle ---------------------
    if (toe_is_error(DBUS_TOE)) {
        MECHANICAL_ARM.error_code |= DBUS_ERROR_OFFSET;
    } else {
        MECHANICAL_ARM.error_code &= ~DBUS_ERROR_OFFSET;
    }

    // Joint error handle ---------------------
    if (fabs(MECHANICAL_ARM.joint_motor[0].fdb.tor) > JOINT_0_MAX_TORQUE) {
        MECHANICAL_ARM.error_code |= JOINT_0_ERROR_OFFSET;
    } else {
        MECHANICAL_ARM.error_code &= ~JOINT_0_ERROR_OFFSET;
    }

    if (fabs(MECHANICAL_ARM.joint_motor[1].fdb.tor) > JOINT_1_MAX_TORQUE) {
        MECHANICAL_ARM.error_code |= JOINT_1_ERROR_OFFSET;
    } else {
        MECHANICAL_ARM.error_code &= ~JOINT_1_ERROR_OFFSET;
    }

    if (fabs(MECHANICAL_ARM.joint_motor[2].fdb.tor) > JOINT_2_MAX_TORQUE) {
        MECHANICAL_ARM.error_code |= JOINT_2_ERROR_OFFSET;
    } else {
        MECHANICAL_ARM.error_code &= ~JOINT_2_ERROR_OFFSET;
    }

    // Zero setted expection handle ---------------------
    if (MECHANICAL_ARM.mode == MECHANICAL_ARM_SET_ZERO) {
        if ((MECHANICAL_ARM.joint_motor[0].fdb.pos < JOINT_ZERO_THRESHOLD) &&
            (MECHANICAL_ARM.joint_motor[1].fdb.pos < JOINT_ZERO_THRESHOLD) &&
            (MECHANICAL_ARM.joint_motor[2].fdb.pos < JOINT_ZERO_THRESHOLD)) {
            MECHANICAL_ARM.zero_setted = true;
        }
    }

    // Position mutation error handle ---------------------
    for (uint8_t i = 0; i < 5; i++) {
        if (fabs(MECHANICAL_ARM.fdb.pos_delta[i]) > POS_MUTATION_THRESHOLD) {
            MECHANICAL_ARM.error_code |= POS_MUTATION_ERROR_OFFSET;
        }
    }
    if (MECHANICAL_ARM.mode == MECHANICAL_ARM_ZERO_FORCE) {
        MECHANICAL_ARM.error_code &= ~POS_MUTATION_ERROR_OFFSET;
    }
}

/*-------------------- Set mode --------------------*/

bool CheckInitCompleted(void);
static MechanicalArmMode_e RemoteControlSetMode(void);
static void SetCtrlLink(void);

/**
 * @brief          设置模式
 * @param[in]      none
 * @retval         none
 */
void MechanicalArmSetMode(void)
{
    if (MECHANICAL_ARM.error_code & DBUS_ERROR_OFFSET) {  // 遥控器出错时的状态处理
        MECHANICAL_ARM.mode = MECHANICAL_ARM_ZERO_FORCE;
        return;
    }

    SetCtrlLink();

    if ((MECHANICAL_ARM.error_code & JOINT_0_ERROR_OFFSET) ||
        (MECHANICAL_ARM.error_code & JOINT_1_ERROR_OFFSET) ||
        (MECHANICAL_ARM.error_code & JOINT_2_ERROR_OFFSET)) {  // 关节出错时的状态处理
        MECHANICAL_ARM.mode = MECHANICAL_ARM_ZERO_FORCE;
        return;
    }

    if (MECHANICAL_ARM.error_code & POS_MUTATION_ERROR_OFFSET) {  // 位置突变时的状态处理
        MECHANICAL_ARM.mode = MECHANICAL_ARM_ZERO_FORCE;
        return;
    }

    if (MECHANICAL_ARM.ctrl_link == LINK_NONE) {
        MECHANICAL_ARM.mode = MECHANICAL_ARM_ZERO_FORCE;
        return;
    }

    MechanicalArmMode_e mode = MECHANICAL_ARM_ZERO_FORCE;

    mode = RemoteControlSetMode();

    if (mode == MECHANICAL_ARM_INIT) {
        if (MECHANICAL_ARM.mode != MECHANICAL_ARM_INIT &&
            MECHANICAL_ARM.mode != MECHANICAL_ARM_SET_ZERO) {  //从其他状态切入
            MECHANICAL_ARM.init_completed[0] = true;
            MECHANICAL_ARM.init_completed[1] = false;
            MECHANICAL_ARM.init_completed[2] = false;
            MECHANICAL_ARM.init_completed[3] = true;
            MECHANICAL_ARM.init_completed[4] = true;
            MECHANICAL_ARM.zero_setted = false;
        }

        if (CheckInitCompleted()) {
            mode = MECHANICAL_ARM_SET_ZERO;
        }
    } else if (mode == MECHANICAL_ARM_FOLLOW) {
        if (!MECHANICAL_ARM.zero_setted) {
            mode = MECHANICAL_ARM_ZERO_FORCE;
        }
    }

    MECHANICAL_ARM.mode = mode;
}

bool CheckInitCompleted(void)
{
    bool init_completed = true;
    for (uint8_t i = 0; i < 5; i++) {
        init_completed = init_completed && MECHANICAL_ARM.init_completed[i];
    }

    if (init_completed) {
        return true;
    }

    if (!MECHANICAL_ARM.init_completed[1]) {  //检测关节1电机初始化状况
        if (fabsf(MECHANICAL_ARM.joint_motor[1].fdb.vel) < JOINT_MIN_VELOCITY &&
            MECHANICAL_ARM.joint_motor[1].fdb.tor >= JOINT_1_INIT_MAX_TORQUE) {
            MECHANICAL_ARM.init_completed[1] = true;
        }
    } else if (!MECHANICAL_ARM.init_completed[2]) {
        if (fabsf(MECHANICAL_ARM.joint_motor[2].fdb.vel) < JOINT_MIN_VELOCITY &&
            MECHANICAL_ARM.joint_motor[2].fdb.tor >= JOINT_2_INIT_MAX_TORQUE) {
            MECHANICAL_ARM.init_completed[2] = true;
        }
    }

    return false;
}

static MechanicalArmMode_e RemoteControlSetMode(void)
{
    if (switch_is_up(MECHANICAL_ARM.rc->rc.s[MECHANICAL_ARM_STATE_CHANNEL])) {
        return MECHANICAL_ARM_FOLLOW;
    } else if (switch_is_mid(MECHANICAL_ARM.rc->rc.s[MECHANICAL_ARM_STATE_CHANNEL])) {
        return MECHANICAL_ARM_INIT;
    } else if (switch_is_down(MECHANICAL_ARM.rc->rc.s[MECHANICAL_ARM_STATE_CHANNEL])) {
        return MECHANICAL_ARM_ZERO_FORCE;
    }
    return MECHANICAL_ARM_ZERO_FORCE;
}

static void SetCtrlLink(void)
{
    if (switch_is_up(MECHANICAL_ARM.rc->rc.s[MECHANICAL_ARM_LINK_CHANNEL])) {
        MECHANICAL_ARM.ctrl_link = LINK_CUSTOM_CONTROLLER;
    } else if (switch_is_mid(MECHANICAL_ARM.rc->rc.s[MECHANICAL_ARM_LINK_CHANNEL])) {
        MECHANICAL_ARM.ctrl_link = LINK_REMOTE_CONTROL;
    } else if (switch_is_down(MECHANICAL_ARM.rc->rc.s[MECHANICAL_ARM_LINK_CHANNEL])) {
        MECHANICAL_ARM.ctrl_link = LINK_NONE;
    }
    MECHANICAL_ARM.ctrl_link = LINK_NONE;
}
/*-------------------- Observe --------------------*/

/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void MechanicalArmObserver(void)
{
    float last_pos[5];
    for (uint8_t i = 0; i < 5; i++) {
        last_pos[i] = MECHANICAL_ARM.fdb.pos[i];
    }

    for (uint8_t i = 0; i < 5; i++) {
        GetMotorMeasure(&MECHANICAL_ARM.joint_motor[i]);
        MECHANICAL_ARM.fdb.pos[i] = MECHANICAL_ARM.joint_motor[i].fdb.pos;
    }
    MECHANICAL_ARM.fdb.pos[0] = theta_transfrom(MECHANICAL_ARM.joint_motor[0].fdb.pos, 0, 1, 1);
    MECHANICAL_ARM.fdb.pos[1] =
        theta_transfrom(MECHANICAL_ARM.joint_motor[1].fdb.pos, J_1_ANGLE_OFFESET, 1, 2);
    MECHANICAL_ARM.fdb.pos[2] =
        theta_transfrom(MECHANICAL_ARM.joint_motor[2].fdb.pos, J_2_ANGLE_OFFESET, -1, 2);
    MECHANICAL_ARM.fdb.pos[3] = theta_transfrom(MECHANICAL_ARM.joint_motor[3].fdb.pos, 0, 1, 1);
    MECHANICAL_ARM.fdb.pos[4] = theta_transfrom(MECHANICAL_ARM.joint_motor[4].fdb.pos, 0, 1, 1);

    for (uint8_t i = 0; i < 5; i++) {
        MECHANICAL_ARM.fdb.pos_delta[i] = MECHANICAL_ARM.fdb.pos[i] - last_pos[i];
    }

    OutputPCData.packets[0].data = MECHANICAL_ARM.joint_motor[0].mode;
    OutputPCData.packets[1].data = MECHANICAL_ARM.joint_motor[1].mode;
    OutputPCData.packets[2].data = MECHANICAL_ARM.joint_motor[2].mode;
    OutputPCData.packets[3].data = MECHANICAL_ARM.joint_motor[0].fdb.pos;
    OutputPCData.packets[4].data = MECHANICAL_ARM.joint_motor[1].fdb.pos;
    OutputPCData.packets[5].data = MECHANICAL_ARM.joint_motor[2].fdb.pos;
    OutputPCData.packets[6].data = MECHANICAL_ARM.joint_motor[0].fdb.tor;
    OutputPCData.packets[7].data = MECHANICAL_ARM.joint_motor[1].fdb.tor;
    OutputPCData.packets[8].data = MECHANICAL_ARM.joint_motor[2].fdb.tor;
    OutputPCData.packets[9].data = MECHANICAL_ARM.mode;
    OutputPCData.packets[10].data = MECHANICAL_ARM.zero_setted;
    OutputPCData.packets[11].data = MECHANICAL_ARM.fdb.pos[0];
    OutputPCData.packets[12].data = MECHANICAL_ARM.fdb.pos[1];
    OutputPCData.packets[13].data = MECHANICAL_ARM.fdb.pos[2];
    OutputPCData.packets[14].data = MECHANICAL_ARM.ref.pos[0];
    OutputPCData.packets[15].data = MECHANICAL_ARM.ref.pos[1];
    OutputPCData.packets[16].data = MECHANICAL_ARM.ref.pos[2];
}

/*-------------------- Reference --------------------*/

/**
 * @brief          更新目标量
 * @param[in]      none
 * @retval         none
 */
void MechanicalArmReference(void)
{
    float yaw = uint_to_float(GetOtherBoardDataUint16(1, 0), -M_PI, M_PI, 16);
    float big_arm_pitch = uint_to_float(GetOtherBoardDataUint16(1, 1), -M_PI_2, M_PI_2, 16);
    float small_arm_pitch = uint_to_float(GetOtherBoardDataUint16(1, 2), -M_PI_2, M_PI_2, 16);
    float small_arm_roll = uint_to_float(GetOtherBoardDataUint16(1, 3), -M_PI, M_PI, 16);

    OutputPCData.packets[17].data = yaw;
    OutputPCData.packets[18].data = big_arm_pitch;
    OutputPCData.packets[19].data = small_arm_pitch;
    OutputPCData.packets[20].data = small_arm_roll;

    MECHANICAL_ARM.ref.pos[0] = MECHANICAL_ARM.rc->rc.ch[4] * RC_TO_ONE * MAX_JOINT_0_POSITION;
    MECHANICAL_ARM.ref.pos[1] = MECHANICAL_ARM.rc->rc.ch[1] * RC_TO_ONE * (-M_PI_2);
    MECHANICAL_ARM.ref.pos[2] = MECHANICAL_ARM.rc->rc.ch[3] * RC_TO_ONE * (-M_PI_2 - 0.6f);
    MECHANICAL_ARM.ref.pos[3] = M_PI_2;
    MECHANICAL_ARM.ref.pos[4] = M_PI_2;

    if (MECHANICAL_ARM.ref.pos[0] > MAX_JOINT_0_POSITION) {
        MECHANICAL_ARM.ref.pos[0] = MAX_JOINT_0_POSITION;
    } else if (MECHANICAL_ARM.ref.pos[0] < MIN_JOINT_0_POSITION) {
        MECHANICAL_ARM.ref.pos[0] = MIN_JOINT_0_POSITION;
    }

    if (MECHANICAL_ARM.ref.pos[1] > MAX_JOINT_1_POSITION) {
        MECHANICAL_ARM.ref.pos[1] = MAX_JOINT_1_POSITION;
    } else if (MECHANICAL_ARM.ref.pos[1] < MIN_JOINT_1_POSITION) {
        MECHANICAL_ARM.ref.pos[1] = MIN_JOINT_1_POSITION;
    }

    if (MECHANICAL_ARM.ref.pos[2] > MAX_JOINT_2_POSITION) {
        MECHANICAL_ARM.ref.pos[2] = MAX_JOINT_2_POSITION;
    } else if (MECHANICAL_ARM.ref.pos[2] < MIN_JOINT_2_POSITION) {
        MECHANICAL_ARM.ref.pos[2] = MIN_JOINT_2_POSITION;
    }

    //关节2与当前关节1的位置比较，不能超过最大差值
    if (MECHANICAL_ARM.ref.pos[2] - MECHANICAL_ARM.fdb.pos[1] > J_1_J_2_DELTA_MAX) {
        MECHANICAL_ARM.ref.pos[2] = MECHANICAL_ARM.fdb.pos[1] + J_1_J_2_DELTA_MAX;
    } else if (MECHANICAL_ARM.ref.pos[2] - MECHANICAL_ARM.fdb.pos[1] < J_1_J_2_DELTA_MIN) {
        MECHANICAL_ARM.ref.pos[2] = MECHANICAL_ARM.fdb.pos[1] + J_1_J_2_DELTA_MIN;
    }

    // EngineerCustomControllerData_t engineer_custom_controller_data;
    // EngineeringCustomControllerRxDecode(&engineer_custom_controller_data);
}

/*-------------------- Console --------------------*/
static void MechanicalArmInitConsole(void);
static void MechanicalArmFollowConsole(void);
static void MechanicalArmZeroForceConsole(void);
static void JointTorqueLimit(void);

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void MechanicalArmConsole(void)
{
    switch (MECHANICAL_ARM.mode) {
        case MECHANICAL_ARM_INIT: {
            MechanicalArmInitConsole();
        } break;
        case MECHANICAL_ARM_FOLLOW: {
            MechanicalArmFollowConsole();
        } break;
        case MECHANICAL_ARM_ZERO_FORCE:
        case MECHANICAL_ARM_SET_ZERO:
        default: {
            MechanicalArmZeroForceConsole();
        }
    }

    JointTorqueLimit();
}

static void MechanicalArmInitConsole(void)
{
    // 0-2关节初始化
    // 关节0无需初始化
    MECHANICAL_ARM.joint_motor[0].set.vel = 0.0f;
    MECHANICAL_ARM.joint_motor[0].set.tor = 0.0f;

    // 先对关节1进行初始化，再对关节2进行初始化
    if (!MECHANICAL_ARM.init_completed[1]) {
        MECHANICAL_ARM.joint_motor[1].set.vel = JOINT_INIT_VELOCITY_SET;
        MECHANICAL_ARM.joint_motor[2].set.vel = 0.0f;

        MECHANICAL_ARM.joint_motor[1].set.tor = 0.0f;
        MECHANICAL_ARM.joint_motor[2].set.tor = 0.0f;
    } else if (!MECHANICAL_ARM.init_completed[2]) {
        MECHANICAL_ARM.joint_motor[1].set.vel = 0.0f;
        MECHANICAL_ARM.joint_motor[2].set.vel = JOINT_INIT_VELOCITY_SET;

        MECHANICAL_ARM.joint_motor[1].set.tor = 0.0f;
        MECHANICAL_ARM.joint_motor[2].set.tor = 0.0f;
    } else {
        MECHANICAL_ARM.joint_motor[1].set.vel = 0.0f;
        MECHANICAL_ARM.joint_motor[2].set.vel = 0.0f;

        MECHANICAL_ARM.joint_motor[1].set.tor = 0.0f;
        MECHANICAL_ARM.joint_motor[2].set.tor = 0.0f;
    }
}

static void MechanicalArmFollowConsole(void)
{
    MECHANICAL_ARM.joint_motor[0].set.pos = MECHANICAL_ARM.ref.pos[0];

    MECHANICAL_ARM.joint_motor[1].set.pos =
        theta_transfrom(MECHANICAL_ARM.ref.pos[1], -J_1_ANGLE_OFFESET, 1, 1);
    MECHANICAL_ARM.joint_motor[1].mode = CYBERGEAR_MODE_POS;

    MECHANICAL_ARM.joint_motor[2].set.pos =
        theta_transfrom(MECHANICAL_ARM.ref.pos[2], J_2_ANGLE_OFFESET, -1, 1);
    MECHANICAL_ARM.joint_motor[2].mode = CYBERGEAR_MODE_POS;
}

static void MechanicalArmZeroForceConsole(void)
{
    MECHANICAL_ARM.joint_motor[0].set.vel = 0.0f;
    MECHANICAL_ARM.joint_motor[1].set.vel = 0.0f;
    MECHANICAL_ARM.joint_motor[2].set.vel = 0.0f;

    MECHANICAL_ARM.joint_motor[0].set.tor = 0.0f;
    MECHANICAL_ARM.joint_motor[1].set.tor = 0.0f;
    MECHANICAL_ARM.joint_motor[2].set.tor = 0.0f;
}

static void JointTorqueLimit(void)
{
    static float max_torque[3] = {JOINT_0_MAX_TORQUE, JOINT_1_MAX_TORQUE, JOINT_2_MAX_TORQUE};
    for (uint8_t i = 0; i <= 2; i++) {
        if (MECHANICAL_ARM.joint_motor[i].fdb.tor > max_torque[i]) {
            MECHANICAL_ARM.joint_motor[i].set.tor = max_torque[i];
            MECHANICAL_ARM.joint_motor[i].mode = CYBERGEAR_MODE_TORQUE;
        } else if (MECHANICAL_ARM.joint_motor[i].fdb.tor < -max_torque[i]) {
            MECHANICAL_ARM.joint_motor[i].set.tor = -max_torque[i];
            MECHANICAL_ARM.joint_motor[i].mode = CYBERGEAR_MODE_TORQUE;
        }
    }
}

/*-------------------- Cmd --------------------*/
static void ArmEnable(void);
static void ArmInitSendCmd(void);
static void ArmSetZeroSendCmd(void);
static void ArmFollowSendCmd(void);
static void ArmZeroForceSendCmd(void);

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */
void MechanicalArmSendCmd(void)
{
    ArmEnable();
    switch (MECHANICAL_ARM.mode) {
        case MECHANICAL_ARM_INIT: {
            ArmInitSendCmd();
        } break;
        case MECHANICAL_ARM_FOLLOW: {
            ArmFollowSendCmd();
        } break;
        case MECHANICAL_ARM_SET_ZERO: {
            ArmSetZeroSendCmd();
        }
        case MECHANICAL_ARM_ZERO_FORCE:
        default: {
            ArmZeroForceSendCmd();
        }
    }
}

static void ArmEnable(void)
{
    if (MECHANICAL_ARM.joint_motor[0].fdb.state == RESET_MODE) {
        CybergearEnable(&MECHANICAL_ARM.joint_motor[0]);
        delay_us(5);
    }
    if (MECHANICAL_ARM.joint_motor[1].fdb.state == RESET_MODE) {
        CybergearEnable(&MECHANICAL_ARM.joint_motor[1]);
        delay_us(5);
    }
    if (MECHANICAL_ARM.joint_motor[2].fdb.state == RESET_MODE) {
        CybergearEnable(&MECHANICAL_ARM.joint_motor[2]);
        delay_us(5);
    }
}

static void ArmInitSendCmd(void)
{
    CybergearVelocityControl(&MECHANICAL_ARM.joint_motor[0], 1);
    for (int i = 0; i < 1; i++) CybergearReadParam(&MECHANICAL_ARM.joint_motor[0], 0X302d);

    CybergearVelocityControl(&MECHANICAL_ARM.joint_motor[1], 4.0);
    for (int i = 0; i < 1; i++) CybergearReadParam(&MECHANICAL_ARM.joint_motor[1], 0X302d);

    CybergearVelocityControl(&MECHANICAL_ARM.joint_motor[2], 2.1);
    for (int i = 0; i < 1; i++) CybergearReadParam(&MECHANICAL_ARM.joint_motor[2], 0X302d);
}

static void ArmSetZeroSendCmd(void)
{
    CybergearSetMechPositionToZero(&MECHANICAL_ARM.joint_motor[0]);
    delay_us(5);
    CybergearSetMechPositionToZero(&MECHANICAL_ARM.joint_motor[1]);
    delay_us(5);
    CybergearSetMechPositionToZero(&MECHANICAL_ARM.joint_motor[2]);
    delay_us(5);
}

static void ArmFollowSendCmd(void)
{
    CybergearPositionControl(&MECHANICAL_ARM.joint_motor[0], 2, 0.5);
    for (int i = 0; i < 1; i++) CybergearReadParam(&MECHANICAL_ARM.joint_motor[0], 0X302d);

    static float kp_vel[3] = {0, 3.0f, 4.0f};
    static float kp_pos[3] = {0, 3.0f, 4.0f};
    for (int i = 1; i <= 2; i++) {
        if (MECHANICAL_ARM.joint_motor[i].mode == CYBERGEAR_MODE_SPEED) {
            CybergearVelocityControl(&MECHANICAL_ARM.joint_motor[i], kp_vel[i]);
        } else if (MECHANICAL_ARM.joint_motor[i].mode == CYBERGEAR_MODE_POS) {
            CybergearPositionControl(&MECHANICAL_ARM.joint_motor[i], kp_pos[i], 0.5);
        } else {
            CybergearTorqueControl(&MECHANICAL_ARM.joint_motor[i]);
        }
        for (int i = 0; i < 1; i++) CybergearReadParam(&MECHANICAL_ARM.joint_motor[1], 0X302d);
    }
}

static void ArmZeroForceSendCmd(void)
{
    MECHANICAL_ARM.joint_motor[0].set.tor = 0;
    MECHANICAL_ARM.joint_motor[1].set.tor = 0;
    MECHANICAL_ARM.joint_motor[2].set.tor = 0;
    CybergearTorqueControl(&MECHANICAL_ARM.joint_motor[0]);
    for (int i = 0; i < 1; i++) CybergearReadParam(&MECHANICAL_ARM.joint_motor[0], 0X302d);

    CybergearTorqueControl(&MECHANICAL_ARM.joint_motor[1]);
    for (int i = 0; i < 1; i++) CybergearReadParam(&MECHANICAL_ARM.joint_motor[1], 0X302d);

    CybergearTorqueControl(&MECHANICAL_ARM.joint_motor[2]);
    for (int i = 0; i < 1; i++) CybergearReadParam(&MECHANICAL_ARM.joint_motor[2], 0X302d);
}

#endif /* MECHANICAL_ARM_5_AXIS */
