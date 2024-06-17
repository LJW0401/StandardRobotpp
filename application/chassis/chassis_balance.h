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
#ifndef CHASSIS_BALANCE_H
#define CHASSIS_BALANCE_H

#include "robot_param.h"

#if (CHASSIS_TYPE == CHASSIS_BALANCE)
#include "IMU_task.h"
#include "chassis.h"
#include "math.h"
#include "motor.h"
#include "pid.h"
#include "remote_control.h"
#include "struct_typedef.h"
#include "user_lib.h"

// clang-format off
#define JOINT_ERROR_OFFSET   ((uint8_t)1 << 0)  // 关节电机错误偏移量
#define WHEEL_ERROR_OFFSET   ((uint8_t)1 << 1)  // 驱动轮电机错误偏移量
#define DBUS_ERROR_OFFSET    ((uint8_t)1 << 2)  // dbus错误偏移量
#define IMU_ERROR_OFFSET     ((uint8_t)1 << 3)  // imu错误偏移量
#define FLOATING_OFFSET      ((uint8_t)1 << 4)  // 悬空状态偏移量
// clang-format on

/*-------------------- Structural definition --------------------*/

typedef enum {
    CHASSIS_OFF,        // 底盘关闭
    CHASSIS_SAFE,       // 底盘无力，所有控制量置0
    CHASSIS_STAND_UP,   // 底盘起立，从倒地状态到站立状态的中间过程
    CHASSIS_CALIBRATE,  // 底盘校准
    CHASSIS_FOLLOW_GIMBAL_YAW,  // 底盘跟随云台（运动方向为云台坐标系方向，需进行坐标转换）
    CHASSIS_FLOATING,    // 底盘悬空状态
    CHASSIS_CUSHIONING,  // 底盘缓冲状态
    CHASSIS_FREE,        // 底盘不跟随云台
    CHASSIS_AUTO,        // 底盘自动模式
    CHASSIS_DEBUG,       // 调试模式
    CHASSIS_CUSTOM       // 自定义模式
} ChassisMode_e;

typedef struct Leg
{
    struct rod
    {
        float Angle;    // rad
        float dAngle;   // rad/s
        float ddAngle;  // rad/s^2

        float Length;    // m
        float dLength;   // m/s
        float ddLength;  // m/s^2

        float F;   // N
        float Tp;  // N*m
    } rod;

    struct joint
    {
        float Angle;   // rad 0-前 1-后
        float dAngle;  // rad/s 0-前 1-后
    } joint[2];

    struct wheel
    {
        float Angle;     // rad
        float Velocity;  // rad/s
    } wheel;
} Leg_t;

/**
 * @brief      比例系数结构体
 * @note       比例系数，用于手动优化控制效果
 */
typedef struct
{
    float k[2][6];
    float Tp;
    float T;
    float length;
} Ratio_t;

/**
 * @brief 状态、期望和限制值
 */
typedef struct
{
    float theta;
    float theta_dot;
    float x;
    float x_dot;
    float phi;
    float phi_dot;

    float speed_integral;
    float roll;
    float roll_velocity;
    float yaw;
    float yaw_velocity;

    Leg_t leg[2];  // 0-左 1-右

    ChassisSpeedVector_t speed_vector;
} Values_t;

typedef struct
{
    pid_type_def yaw_angle;
    pid_type_def yaw_velocity;

    pid_type_def vel_add;

#if LOCATION_CONTROL
    pid_type_def roll_angle;

    pid_type_def pitch_angle;
    pid_type_def pitch_vel;
#else

    pid_type_def roll_angle;
    // pid_type_def roll_velocity;

    pid_type_def pitch_angle;
    // pid_type_def pitch_vel;

    pid_type_def leg_length_length[2];
    pid_type_def leg_length_speed[2];

    pid_type_def leg_angle_angle;
#endif

    pid_type_def stand_up;
    pid_type_def wheel_stop[2];
} PID_t;

typedef struct LPF
{
    LowPassFilter_t leg_length_accel_filter[2];
    LowPassFilter_t leg_angle_accel_filter[2];
    LowPassFilter_t support_force_filter[2];
} LPF_t;

/**
 * @brief  底盘数据结构体
 * @note   底盘坐标使用右手系，前进方向为x轴，左方向为y轴，上方向为z轴
 */
typedef struct
{
    const RC_ctrl_t * rc;  // 底盘使用的遥控器指针
    const Imu_t * imu;     // imu数据
    ChassisMode_e mode;    // 底盘模式
    ChassisState_e state;  // 底盘状态
    uint8_t error_code;    // 底盘错误代码

    /*-------------------- Motors --------------------*/
    // 平衡底盘有2个驱动轮电机和4个关节电机
    Motor_s joint_motor[4];
    Motor_s wheel_motor[2];  // 驱动轮电机 0-左轮，1-右轮
    /*-------------------- Values --------------------*/

    Values_t ref;  // 期望值
    Values_t fdb;  // 状态值

    PID_t pid;  // PID控制器
    LPF_t lpf;  // 低通滤波器

    Ratio_t ratio;  // 比例系数

    float dyaw;  // (rad)(feedback)当前位置与云台中值角度差（用于坐标转换）
    uint16_t yaw_mid;  // (ecd)(preset)云台中值角度
} Chassis_s;

typedef struct Calibrate
{
    uint32_t cali_cnt;      //记录遥控器摇杆保持校准姿态的次数（等效于时间）
    float velocity[4];      //关节电机速度
    uint32_t stpo_time[4];  //停止时间
    bool reached[4];        //是否到达限位
    bool calibrated;        //完成校准
} Calibrate_s;

typedef struct GroundTouch
{
    uint32_t touch_time;     //记录触地时间
    float force[2];          //记录腿上的力
    float support_force[2];  //(N)地面的支持力 0-左 1-右

    bool touch;  //是否触地
} GroundTouch_s;

extern void ChassisInit(void);

extern void ChassisHandleException(void);

extern void ChassisSetMode(void);

extern void ChassisObserver(void);

extern void ChassisReference(void);

extern void ChassisConsole(void);

extern void ChassisSendCmd(void);

#endif /* CHASSIS_BALANCE */
#endif /* CHASSIS_BALANCE_H */
