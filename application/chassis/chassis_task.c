/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       chassis_task.c/h
  * @brief      chassis control task,
  *             底盘控制任务
  * @note
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
#include "chassis_task.h"
#include "chassis_console.h"
#include "chassis_calculate.h"

#include "cmsis_os.h"

#include "arm_math.h"
#include "pid.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "IMU_task.h"
#include "chassis_power_control.h"

#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

static void InitChassis(Chassis_s *chassis);

static void SetChassisMode(Chassis_s *chassis);

static void ChassisObserver(Chassis_s *chassis);

static void ChassisReference(Chassis_s *chassis);

static void ChassisConsole(Chassis_s *chassis);

static void SendChassisCmd(Chassis_s *chassis);

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif

/**
 * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
 * @param[in]      pvParameters: 空
 * @retval         none
 */
void chassis_task(void const *pvParameters)
{
    // 空闲一段时间
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    // 初始化底盘
    InitChassis(&chassis);

    while (1)
    {
        // 设置底盘模式
        SetChassisMode(&chassis);
        // 更新状态量
        ChassisObserver(&chassis);
        // 更新目标量
        ChassisReference(&chassis);
        // 计算控制量
        ChassisConsole(&chassis);
        // 发送控制量
        SendChassisCmd(&chassis);
        // 系统延时
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/**
 * @brief          初始化
 * @param[in]      chassis 底盘结构体指针
 * @retval         none
 */
static void InitChassis(Chassis_s *chassis)
{
    if (chassis == NULL)
    {
        return;
    }
    chassis->rc = get_remote_control_point(); // 获取遥控器指针

    /*-------------------- 初始化底盘电机 --------------------*/
#if (CHASSIS_TYPE == CHASSIS_MECANUM_WHEEL)
    void InitMecanumChassis(Chassis_s * chassis);
#elif (CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)
    void InitOmniChassis(Chassis_s * chassis);
#elif (CHASSIS_TYPE == CHASSIS_STEERING_WHEEL)
    void InitSteeringChassis(Chassis_s * chassis);
#elif (CHASSIS_TYPE == CHASSIS_BALANCE)
    void InitBalanceChassisMotor(Chassis_s * chassis);
#endif
}

/**
 * @brief          设置模式
 * @param[in]      chassis 底盘结构体指针
 * @retval         none
 */
static void SetChassisMode(Chassis_s *chassis)
{
    if (chassis == NULL)
    {
        return;
    }

    if (switch_is_up(chassis->rc->rc.s[CHASSIS_MODE_CHANNEL]))
    {
        chassis->mode = CHASSIS_AUTO;
    }
    else if (switch_is_mid(chassis->rc->rc.s[CHASSIS_MODE_CHANNEL]))
    {
        chassis->mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    }
    else if (switch_is_down(chassis->rc->rc.s[CHASSIS_MODE_CHANNEL]))
    {
        chassis->mode = CHASSIS_FREE;
    }
}

/**
 * @brief          更新状态量
 * @param[in]      chassis 底盘结构体指针
 * @retval         none
 */
static void ChassisObserver(Chassis_s *chassis)
{
    if (chassis == NULL)
    {
        return;
    }

    //uint8_t i;

#if (CHASSIS_TYPE == CHASSIS_BALANCE) // 平衡底盘和其他底盘不太一样
    // 更新底盘陀螺仪数据
    double left_leg_pos[2];
    double right_leg_pos[2];
    LegFKine(0, 0, left_leg_pos);  // 获取五连杆等效连杆长度
    LegFKine(0, 0, right_leg_pos); // 获取五连杆等效连杆长度

    // LegPosUpdate();

    chassis->feedback.x[0] = chassis->imu->angle.pitch;
    chassis->feedback.x[1] = chassis->imu->velocity.y;
    chassis->feedback.x[2] = 0;
    // chassis->feedback->x[3] = (left_wheel.speed + right_wheel.speed) / 2 * WHEEL_RADIUS;
    chassis->feedback.x[4] = (chassis->feedback.leg_pos[0].angle + chassis->feedback.leg_pos[1].angle) / 2 - M_PI_2 - chassis->imu->angle.pitch;
    chassis->feedback.x[5] = 0;
#else
    for (i = 0; i < 4; i++)
    {
        chassis->motor[i].w = chassis->motor[i].motor_measure->speed_rpm * DJI_GM3508_RPM_TO_OMEGA;
        chassis->motor[i].v = chassis->motor[i].w * WHEEL_RADIUS;
    }
#endif
    chassis->dyaw = (chassis->yaw_motor->motor_measure->ecd * DJI_GM6020_ECD_TO_RAD - chassis->yaw_mid);
}

/**
 * @brief          更新目标量
 * @param[in]      chassis 底盘结构体指针
 */
static void ChassisReference(Chassis_s *chassis)
{
    if (chassis == NULL)
    {
        return;
    }

    int16_t rc_x = 0, rc_y = 0, rc_wz = 0;
    rc_deadband_limit(chassis->rc->rc.ch[CHASSIS_X_CHANNEL], rc_x, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis->rc->rc.ch[CHASSIS_Y_CHANNEL], rc_y, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis->rc->rc.ch[CHASSIS_WZ_CHANNEL], rc_wz, CHASSIS_RC_DEADLINE);

    ChassisSpeedVector_t v_set = {0.0f, 0.0f, 0.0f};
    if (chassis->mode == CHASSIS_FREE) // 底盘自由模式下，控制量为底盘坐标系下的速度
    {
        v_set.vx = rc_x * RC_TO_ONE * chassis->upper_limit.speed_vector.vx;
        v_set.vy = rc_y * RC_TO_ONE * chassis->upper_limit.speed_vector.vy;
        v_set.wz = rc_wz * RC_TO_ONE * chassis->upper_limit.speed_vector.wz;
    }
    else if (chassis->mode == CHASSIS_FOLLOW_GIMBAL_YAW) // 云台跟随模式下，控制量为云台坐标系下的速度，需要进行坐标转换
    {
        v_set.vx = rc_x * RC_TO_ONE * chassis->upper_limit.speed_vector.vx;
        v_set.vy = rc_y * RC_TO_ONE * chassis->upper_limit.speed_vector.vy;
        v_set.wz = rc_wz * RC_TO_ONE * chassis->upper_limit.speed_vector.wz;
        GimbalSpeedVectorToChassisSpeedVector(&v_set, chassis->dyaw);
    }
    else if (chassis->mode == CHASSIS_AUTO) // 底盘自动模式，控制量为云台坐标系下的速度，需要进行坐标转换
    {
        // TODO: add code here
    }
    chassis->reference.speed_vector.vx = v_set.vx;
    chassis->reference.speed_vector.vy = v_set.vy;
    chassis->reference.speed_vector.wz = v_set.wz;

#if (CHASSIS_TYPE == CHASSIS_BALANCE)
    chassis->reference.x[0] = 0;
    chassis->reference.x[1] = 0;
    chassis->reference.x[2] = 0;
    chassis->reference.x[3] = chassis->reference.speed_vector.vx;
    chassis->reference.x[4] = 0;
    chassis->reference.x[5] = 0;
#endif
}

/**
 * @brief          计算控制量
 * @param[in]      chassis 底盘结构体指针
 * @retval         none
 */
static void ChassisConsole(Chassis_s *chassis)
{
    if (chassis == NULL)
    {
        return;
    }

#if (CHASSIS_TYPE == CHASSIS_MECANUM_WHEEL)

#elif (CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)

#elif (CHASSIS_TYPE == CHASSIS_STEERING_WHEEL)

#elif (CHASSIS_TYPE == CHASSIS_BALANCE)
    BalanceChassisConsole(chassis);
#endif
}

/**
 * @brief          发送控制量
 * @param[in]      chassis 底盘结构体指针
 * @retval         none
 */
static void SendChassisCmd(Chassis_s *chassis)
{
    if (chassis == NULL)
    {
        return;
    }

#if (CHASSIS_TYPE == CHASSIS_MECANUM_WHEEL)

#elif (CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)

#elif (CHASSIS_TYPE == CHASSIS_STEERING_WHEEL)

#elif (CHASSIS_TYPE == CHASSIS_BALANCE)
    SendBalanceChassisCmd(chassis);
#endif
}
