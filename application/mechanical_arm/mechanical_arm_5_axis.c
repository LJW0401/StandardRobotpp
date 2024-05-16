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

#include "mechanical_arm_5_axis.h"

#if (MECHANICAL_ARM_TYPE == MECHANICAL_ARM_5_AXIS)
#include <stdbool.h>

#include "CAN_communication.h"
#include "custom_controller_connect.h"
#include "detect_task.h"
#include "math.h"
#include "pid.h"
#include "signal_generator.h"
#include "usb_task.h"
#include "user_lib.h"

static MechanicalArm_s MECHANICAL_ARM = {
    .mode = MECHANICAL_ARM_ZERO_FORCE,
    .reference =
        {
            .position = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        },
    .feedback =
        {
            .position = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        },
    .upper_limit =
        {
            .position =
                {MAX_JOINT_0_POSITION, MAX_JOINT_1_POSITION, MAX_JOINT_2_POSITION,
                 MAX_JOINT_3_POSITION, MAX_JOINT_4_POSITION},
        },
    .lower_limit =
        {
            .position =
                {MIN_JOINT_0_POSITION, MIN_JOINT_1_POSITION, MIN_JOINT_2_POSITION,
                 MIN_JOINT_3_POSITION, MIN_JOINT_4_POSITION},
        },
    .init_completed = {false, false, false, false, false},
};

Motor_s dm_motor = {
    .can = 1,
    .id = 1,
    .direction = 1,
    .reduction_ratio = 1,
    .type = DM_8009,
    .mode = DM_MODE_MIT,
    .set = {.torque = 1, .velocity = 1, .position = 0},
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
        &MECHANICAL_ARM.joint_motor[0], JOINT_MOTOR_0_ID, JOINT_MOTOR_0_CAN, JOINT_MOTOR_0_TYPE,
        JOINT_MOTOR_0_DIRECTION, JOINT_MOTOR_0_REDUCTION_RATIO, JOINT_MOTOR_0_MODE);
    MotorInit(
        &MECHANICAL_ARM.joint_motor[1], JOINT_MOTOR_1_ID, JOINT_MOTOR_1_CAN, JOINT_MOTOR_1_TYPE,
        JOINT_MOTOR_1_DIRECTION, JOINT_MOTOR_1_REDUCTION_RATIO, JOINT_MOTOR_1_MODE);
    MotorInit(
        &MECHANICAL_ARM.joint_motor[2], JOINT_MOTOR_2_ID, JOINT_MOTOR_2_CAN, JOINT_MOTOR_2_TYPE,
        JOINT_MOTOR_2_DIRECTION, JOINT_MOTOR_2_REDUCTION_RATIO, JOINT_MOTOR_2_MODE);
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
        return;
    }

    if (MECHANICAL_ARM.mode < MECHANICAL_ARM_INIT) {
        MECHANICAL_ARM.mode = MECHANICAL_ARM_INIT;
    } else if (MECHANICAL_ARM.mode == MECHANICAL_ARM_INIT) {
        if (CheckInitCompleted()) {
            MECHANICAL_ARM.mode = MECHANICAL_ARM_FOLLOW;
        }
    }
}

bool CheckInitCompleted(void)
{
    bool init_completed = true;
    for (uint8_t i = 1; i < 5; i++) {
        init_completed = init_completed && MECHANICAL_ARM.init_completed[i];
    }

    if (init_completed) {
        return true;
    }

    if (!MECHANICAL_ARM.init_completed[1]) {  //检测关节1电机初始化状况
        if (fabsf(MECHANICAL_ARM.joint_motor[1].fdb.w) < JOINT_MIN_VELOCITY &&
            MECHANICAL_ARM.joint_motor[1].fdb.T >= JOINT_INIT_MAX_TORQUE) {
            MECHANICAL_ARM.init_completed[1] = true;
        }
    }

    // TODO: add init completed condition
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
    for (uint8_t i = 1; i < 5; i++) {
        GetMotorMeasure(&MECHANICAL_ARM.joint_motor[i]);
        MECHANICAL_ARM.feedback.position[i] = MECHANICAL_ARM.joint_motor[i].fdb.pos;
    }
    MECHANICAL_ARM.feedback.position[0] =
        theta_transfrom(MECHANICAL_ARM.joint_motor[0].fdb.pos, 0, 1);
    MECHANICAL_ARM.feedback.position[1] =
        theta_transfrom(MECHANICAL_ARM.joint_motor[1].fdb.pos, 0, 1);
    MECHANICAL_ARM.feedback.position[2] =
        theta_transfrom(MECHANICAL_ARM.joint_motor[2].fdb.pos, 0, 1);
    MECHANICAL_ARM.feedback.position[3] =
        theta_transfrom(MECHANICAL_ARM.joint_motor[3].fdb.pos, 0, 1);
    MECHANICAL_ARM.feedback.position[4] =
        theta_transfrom(MECHANICAL_ARM.joint_motor[4].fdb.pos, 0, 1);

    GetMotorMeasure(&dm_motor);
    OutputPCData.data_2 = dm_motor.fdb.pos;
}

/*-------------------- Reference --------------------*/

/**
 * @brief          更新目标量
 * @param[in]      none
 * @retval         none
 */
void MechanicalArmReference(void)
{
    MECHANICAL_ARM.reference.position[0] = 0.0f;
    MECHANICAL_ARM.reference.position[1] = M_PI_2;
    MECHANICAL_ARM.reference.position[2] = 0.0f;
    MECHANICAL_ARM.reference.position[3] = M_PI_2;
    MECHANICAL_ARM.reference.position[4] = M_PI_2;

    EngineerCustomControllerData_t engineer_custom_controller_data;
    EngineeringCustomControllerRxDecode(&engineer_custom_controller_data);

    dm_motor.set.position = GenerateSinWave(1.0f, 0.0f, 2.0f);
    OutputPCData.data_1 = dm_motor.set.position;
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
            MECHANICAL_ARM.joint_motor[0].set.velocity = 0.0f;
            MECHANICAL_ARM.joint_motor[0].set.torque = 0.0f;
            // 先对关节1进行初始化，再对关节2进行初始化
            if (!MECHANICAL_ARM.init_completed[1]) {
                MECHANICAL_ARM.joint_motor[1].set.velocity = JOINT_INIT_VELOCITY_SET;
                MECHANICAL_ARM.joint_motor[2].set.velocity = 0.0f;

                MECHANICAL_ARM.joint_motor[1].set.torque = 0.0f;
                MECHANICAL_ARM.joint_motor[2].set.torque = 0.0f;
            } else if (!MECHANICAL_ARM.init_completed[2]) {
                MECHANICAL_ARM.joint_motor[1].set.velocity = 0.0f;
                MECHANICAL_ARM.joint_motor[2].set.velocity = JOINT_INIT_VELOCITY_SET;

                // 关节1初始化完成后，对关节1进行力矩控制，压住关节1防止关节1移动
                MECHANICAL_ARM.joint_motor[1].set.torque = JOINT_1_INIT_TORQUE_SET;
                MECHANICAL_ARM.joint_motor[2].set.torque = 0.0f;
            }
            // 3-4关节初始化（同步进行）
        } break;

        default:
            break;
    }

#define STOP_ALL_JOINT
#ifdef STOP_ALL_JOINT
    MECHANICAL_ARM.joint_motor[0].set.velocity = 0.0f;
    MECHANICAL_ARM.joint_motor[1].set.velocity = 0.0f;
    MECHANICAL_ARM.joint_motor[2].set.velocity = 0.0f;

    MECHANICAL_ARM.joint_motor[0].set.torque = 0.0f;
    MECHANICAL_ARM.joint_motor[1].set.torque = 0.0f;
    MECHANICAL_ARM.joint_motor[2].set.torque = 0.0f;

    MECHANICAL_ARM.joint_motor[3].set.current = 0.0f;
    MECHANICAL_ARM.joint_motor[4].set.current = 0.0f;
#endif
}

/*-------------------- Cmd --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */
void SendMechanicalArmCmd(void)
{
    switch (MECHANICAL_ARM.mode) {
        case MECHANICAL_ARM_INIT: {
            CybergearVelocityControl(&MECHANICAL_ARM.joint_motor[0], 0.5);
            CybergearVelocityControl(&MECHANICAL_ARM.joint_motor[1], 0.5);
            CybergearVelocityControl(&MECHANICAL_ARM.joint_motor[2], 0.5);
            CanCmdDjiMotor(2, DJI_2FF, MECHANICAL_ARM.joint_motor[3].set.current, 0, 0, 0);
            CanCmdDjiMotor(2, DJI_200, 0, MECHANICAL_ARM.joint_motor[4].set.current, 0, 0);
        } break;
        case MECHANICAL_ARM_FOLLOW: {
            CybergearVelocityControl(&MECHANICAL_ARM.joint_motor[0], 0.5);
            CybergearVelocityControl(&MECHANICAL_ARM.joint_motor[1], 0.5);
            CybergearVelocityControl(&MECHANICAL_ARM.joint_motor[2], 0.5);
            CanCmdDjiMotor(2, DJI_2FF, MECHANICAL_ARM.joint_motor[3].set.current, 0, 0, 0);
            CanCmdDjiMotor(2, DJI_200, 0, MECHANICAL_ARM.joint_motor[4].set.current, 0, 0);
        } break;
        case MECHANICAL_ARM_ZERO_FORCE:
        default: {
            CybergearTorqueControl(&MECHANICAL_ARM.joint_motor[0]);
            CybergearTorqueControl(&MECHANICAL_ARM.joint_motor[1]);
            CybergearTorqueControl(&MECHANICAL_ARM.joint_motor[2]);
            CanCmdDjiMotor(2, DJI_2FF, 0, 0, 0, 0);
            CanCmdDjiMotor(2, DJI_200, 0, 0, 0, 0);
        }
    }

    DmEnable(&dm_motor);
    // DmMitCtrlTorque(&dm_motor);
    DmMitCtrlPosition(&dm_motor, 2, 1);
}

#endif /* MECHANICAL_ARM_5_AXIS */
