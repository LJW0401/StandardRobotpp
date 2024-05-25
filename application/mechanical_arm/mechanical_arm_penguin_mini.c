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
void InitMechanicalArm(void)
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

/*-------------------- Set mode --------------------*/

bool CheckInitCompleted(void);

/**
 * @brief          设置模式
 * @param[in]      none
 * @retval         none
 */
void SetMechanicalArmMode(void)
{
    if (toe_is_error(DBUS_TOE)) {
        MECHANICAL_ARM.mode = MECHANICAL_ARM_ZERO_FORCE;
        return;
    }

    if (MECHANICAL_ARM.mode < MECHANICAL_ARM_INIT) {
        MECHANICAL_ARM.mode = MECHANICAL_ARM_INIT;
    } else if (MECHANICAL_ARM.mode == MECHANICAL_ARM_INIT) {
        if (CheckInitCompleted()) {
            MECHANICAL_ARM.mode = MECHANICAL_ARM_SET_ZERO;
        }
    } else if (MECHANICAL_ARM.mode == MECHANICAL_ARM_SET_ZERO) {
        if ((MECHANICAL_ARM.joint_motor[0].fdb.pos < JOINT_ZERO_THRESHOLD) &&
            (MECHANICAL_ARM.joint_motor[1].fdb.pos < JOINT_ZERO_THRESHOLD) &&
            (MECHANICAL_ARM.joint_motor[2].fdb.pos < JOINT_ZERO_THRESHOLD)) {
            MECHANICAL_ARM.mode = MECHANICAL_ARM_FOLLOW;
        }
    }
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

/*-------------------- Observe --------------------*/

/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void MechanicalArmObserver(void)
{
    for (uint8_t i = 0; i < 5; i++) {
        GetMotorMeasure(&MECHANICAL_ARM.joint_motor[i]);
        MECHANICAL_ARM.fdb.pos[i] = MECHANICAL_ARM.joint_motor[i].fdb.pos;
    }
    MECHANICAL_ARM.fdb.pos[0] = theta_transfrom(MECHANICAL_ARM.joint_motor[0].fdb.pos, 0, 1, 1);
    MECHANICAL_ARM.fdb.pos[1] = theta_transfrom(MECHANICAL_ARM.joint_motor[1].fdb.pos, 0, 1, 2);
    MECHANICAL_ARM.fdb.pos[2] =
        theta_transfrom(MECHANICAL_ARM.joint_motor[2].fdb.pos, J_2_ANGLE_OFFESET, -1, 2);
    MECHANICAL_ARM.fdb.pos[3] = theta_transfrom(MECHANICAL_ARM.joint_motor[3].fdb.pos, 0, 1, 1);
    MECHANICAL_ARM.fdb.pos[4] = theta_transfrom(MECHANICAL_ARM.joint_motor[4].fdb.pos, 0, 1, 1);

    OutputPCData.packets[0].data = MECHANICAL_ARM.joint_motor[0].fdb.state;
    OutputPCData.packets[1].data = MECHANICAL_ARM.joint_motor[1].fdb.state;
    OutputPCData.packets[2].data = MECHANICAL_ARM.joint_motor[2].fdb.state;
    OutputPCData.packets[3].data = MECHANICAL_ARM.joint_motor[0].fdb.pos;
    OutputPCData.packets[4].data = MECHANICAL_ARM.joint_motor[1].fdb.pos;
    OutputPCData.packets[5].data = MECHANICAL_ARM.joint_motor[2].fdb.pos;
    OutputPCData.packets[6].data = MECHANICAL_ARM.joint_motor[0].fdb.tor;
    OutputPCData.packets[7].data = MECHANICAL_ARM.joint_motor[1].fdb.tor;
    OutputPCData.packets[8].data = MECHANICAL_ARM.joint_motor[2].fdb.tor;
    OutputPCData.packets[9].data = MECHANICAL_ARM.mode;
    OutputPCData.packets[10].data = MECHANICAL_ARM.mode;
    OutputPCData.packets[11].data = MECHANICAL_ARM.fdb.pos[0];
    OutputPCData.packets[12].data = MECHANICAL_ARM.fdb.pos[1];
    OutputPCData.packets[13].data = MECHANICAL_ARM.fdb.pos[2];
}

/*-------------------- Reference --------------------*/

/**
 * @brief          更新目标量
 * @param[in]      none
 * @retval         none
 */
void MechanicalArmReference(void)
{
    MECHANICAL_ARM.ref.pos[0] = 0.0f;
    MECHANICAL_ARM.ref.pos[1] = M_PI_2;
    MECHANICAL_ARM.ref.pos[2] = 0.0f;
    MECHANICAL_ARM.ref.pos[3] = M_PI_2;
    MECHANICAL_ARM.ref.pos[4] = M_PI_2;

    EngineerCustomControllerData_t engineer_custom_controller_data;
    EngineeringCustomControllerRxDecode(&engineer_custom_controller_data);
}

/*-------------------- Console --------------------*/

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void MechanicalArmConsole(void)
{
    switch (MECHANICAL_ARM.mode) {
        case MECHANICAL_ARM_INIT: {
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
        } break;
        case MECHANICAL_ARM_ZERO_FORCE:
        case MECHANICAL_ARM_SET_ZERO:
        default: {
            MECHANICAL_ARM.joint_motor[0].set.vel = 0.0f;
            MECHANICAL_ARM.joint_motor[1].set.vel = 0.0f;
            MECHANICAL_ARM.joint_motor[2].set.vel = 0.0f;

            MECHANICAL_ARM.joint_motor[0].set.tor = 0.0f;
            MECHANICAL_ARM.joint_motor[1].set.tor = 0.0f;
            MECHANICAL_ARM.joint_motor[2].set.tor = 0.0f;
        }
    }
}

/*-------------------- Cmd --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */
void SendMechanicalArmCmd(void)
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

    switch (MECHANICAL_ARM.mode) {
        case MECHANICAL_ARM_INIT: {
            CybergearVelocityControl(&MECHANICAL_ARM.joint_motor[0], 1);
            for (int i = 0; i < 1; i++) CybergearReadParam(&MECHANICAL_ARM.joint_motor[0], 0X302d);

            CybergearVelocityControl(&MECHANICAL_ARM.joint_motor[1], 4.0);
            for (int i = 0; i < 1; i++) CybergearReadParam(&MECHANICAL_ARM.joint_motor[1], 0X302d);

            CybergearVelocityControl(&MECHANICAL_ARM.joint_motor[2], 1.5);
            for (int i = 0; i < 1; i++) CybergearReadParam(&MECHANICAL_ARM.joint_motor[2], 0X302d);
        } break;
        case MECHANICAL_ARM_SET_ZERO: {
            CybergearSetMechPositionToZero(&MECHANICAL_ARM.joint_motor[0]);
            delay_us(5);
            CybergearSetMechPositionToZero(&MECHANICAL_ARM.joint_motor[1]);
            delay_us(5);
            CybergearSetMechPositionToZero(&MECHANICAL_ARM.joint_motor[2]);
            delay_us(5);
        }  //break;
        case MECHANICAL_ARM_FOLLOW: {
        }  //break;
        case MECHANICAL_ARM_ZERO_FORCE:
        default: {
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
    }
}

#endif /* MECHANICAL_ARM_5_AXIS */
