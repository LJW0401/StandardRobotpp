/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       chassis.c/h
  * @brief      底盘控制任务所需要的变量和函数
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

#ifndef CHASSIS_H
#define CHASSIS_H

// 底盘任务相关宏定义
#define CHASSIS_TASK_INIT_TIME 357   // 任务开始空闲一段时间
#define CHASSIS_CONTROL_TIME_MS 2    // 底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME 0.002f  // 底盘任务控制间隔 0.002s

// 底盘的遥控器相关宏定义
#define CHASSIS_MODE_CHANNEL 0  // 选择底盘状态 开关通道号
#define CHASSIS_X_CHANNEL 1     // 前后的遥控器通道号码
#define CHASSIS_Y_CHANNEL 0     // 左右的遥控器通道号码
#define CHASSIS_WZ_CHANNEL 4    // 旋转的遥控器通道号码
#define CHASSIS_RC_DEADLINE 10  // 摇杆死区

#define rc_deadband_limit(input, output, dealine)          \
    {                                                      \
        if ((input) > (dealine) || (input) < -(dealine)) { \
            (output) = (input);                            \
        } else {                                           \
            (output) = 0;                                  \
        }                                                  \
    }

typedef enum {
    CHASSIS_ZERO_FORCE,  // 底盘无力，所有控制量置0
    CHASSIS_FOLLOW_GIMBAL_YAW,  // 底盘跟随云台（运动方向为云台坐标系方向，需进行坐标转换）
    CHASSIS_STOP,  // 底盘停止运动(速度为0)
    CHASSIS_FREE,  // 底盘不跟随云台
    CHASSIS_SPIN,  // 底盘小陀螺模式
    CHASSIS_AUTO,  // 底盘自动模式
    CHASSIS_OPEN   // 遥控器的值乘以比例成电流值开环控制
} ChassisMode_e;

typedef struct  // 底盘速度向量结构体
{
    float vx;  // (m/s) x方向速度
    float vy;  // (m/s) y方向速度
    float wz;  // (rad/s) 旋转速度
} ChassisSpeedVector_t;

extern void GimbalSpeedVectorToChassisSpeedVector(ChassisSpeedVector_t * speed_vector_set, float dyaw);

#endif  // CHASSIS_H
