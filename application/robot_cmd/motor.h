/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       motor.c/h
  * @brief      电机相关部分定义
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024      Penguin         1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */
#ifndef MOTOR_H
#define MOTOR_H

#include "pid.h"
#include "robot_typedef.h"
#include "struct_typedef.h"

#define RPM_TO_OMEGA 0.1047197551f  // (1/60*2*pi) (rpm)->(rad/s)

/*-------------------- DJI Motor --------------------*/

// 电机编码值规整 0—8191
#define ecd_format(ecd)         \
    {                           \
        if ((ecd) > ECD_RANGE)  \
            (ecd) -= ECD_RANGE; \
        else if ((ecd) < 0)     \
            (ecd) += ECD_RANGE; \
    }

#define DJI_GM6020_ECD_TO_RAD 0.000766990394f  // (2*pi/8192) 电机编码器值转换为弧度
#define DJI_GM3508_RPM_TO_OMEGA 0.0055115661f  // (1/60*2*pi/19) m3508(减速比19:1) (rpm)->(rad/s)
#define DJI_GM2006_RPM_TO_OMEGA 0.0029088821f  // (1/60*2*pi/36) m2006(减速比36:1) (rpm)->(rad/s)

typedef struct _DjiMotorMeasure
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} DjiMotorMeasure_t;

/*-------------------- CyberGear --------------------*/
#define CYBERGEAR_NUM 5

typedef enum _CybergearModeState {
    UNDEFINED_MODE = -1,  //未定义模式
    RESET_MODE = 0,       //Reset模式[复位]
    CALI_MODE = 1,        //Cali 模式[标定]
    RUN_MODE = 2          //Motor模式[运行]
} CybergearModeState_e;   //电机模式状态

typedef struct
{
    uint32_t info : 24;
    uint32_t communication_type : 5;
    uint32_t res : 3;
} __attribute__((packed)) RxCanInfo_s;  // 解码内容缓存

typedef struct
{
    uint32_t FE : 8;
    uint32_t motor_id : 16;
    uint32_t communication_type : 5;
    uint32_t res : 3;
    uint32_t MCU_id;
} __attribute__((packed)) RxCanInfoType_0_s;  // 通信类型0解码内容

typedef struct
{
    uint32_t master_can_id : 8;
    uint32_t motor_id : 8;
    uint32_t under_voltage_fault : 1;
    uint32_t over_current_fault : 1;
    uint32_t over_temperature_fault : 1;
    uint32_t magnetic_encoding_fault : 1;
    uint32_t HALL_encoding_failure : 1;
    uint32_t unmarked : 1;
    uint32_t mode_state : 2;
    uint32_t communication_type : 5;
    uint32_t res : 3;
} __attribute__((packed)) RxCanInfoType_2_s;  // 通信类型2解码内容

typedef struct
{
    RxCanInfo_s ext_id;
    uint8_t rx_data[8];
} CybergearMeasure_s;

/*-------------------- DM Motor --------------------*/
#define DM_NUM 6

// clang-format off
#define DM_MODE_MIT     0x000
#define DM_MODE_POS     0x100
#define DM_MODE_SPEED   0x200
#define DM_MODE_POSI    0x300

#define DM_STATE_ENABLE                 0x00
#define DM_STATE_DISABLE                0x01
#define DM_STATE_OVERVOLTAGE            0x08
#define DM_STATE_UNDERVOLTAGE           0x09
#define DM_STATE_OVERCURRENT            0x0A
#define DM_STATE_MOS_OVER_TEMPERATURE   0x0B
#define DM_STATE_COIL_OVER_TEMPERATURE  0x0C
#define DM_STATE_COMMUNICATION_LOSS     0x0D
#define DM_STATE_OVERLOAD               0x0E

#define DM_P_MIN   -12.5f
#define DM_P_MAX    12.5f
#define DM_V_MIN   -30.0f
#define DM_V_MAX    30.0f
#define DM_KP_MIN   0.0f
#define DM_KP_MAX   500.0f
#define DM_KD_MIN   0.0f
#define DM_KD_MAX   5.0f
#define DM_T_MIN   -10.0f
#define DM_T_MAX    10.0f
// clang-format on

typedef struct
{
    int id;
    int state;
    int p_int;
    int v_int;
    int t_int;
    int kp_int;
    int kd_int;

    float pos;
    float vel;
    float tor;
    float Kp;
    float Kd;

    float t_mos;
    float t_rotor;
} DmMeasure_s;

/*-------------------- MF Motor --------------------*/

/*-------------------- Motor struct --------------------*/

/**
 * @brief  通用电机结构体
 * @note   包括电机的信息、状态量和控制量
 */
typedef struct __Motor
{
    /*电机信息*/
    uint8_t id;             // 电机ID
    uint8_t can;            // 电机所用CAN口
    MotorType_e type;       // 电机类型
    int8_t direction;       // 电机旋转方向（1或-1）
    float reduction_ratio;  // 电机减速比
    uint16_t mode;          // 电机模式

    /*状态量*/
    struct __fdb
    {
        float a;            // (rad/s^2)电机加速度
        float w;            // (rad/s)电机输出轴转速
        float T;            // (N*m)电机输出力矩
        float pos;          // (rad)电机输出轴位置
        float temperature;  // (℃)电机温度
        float current;      // (A)电机电流
        int16_t round;      // (r)电机旋转圈数(用于计算输出轴位置)
        uint16_t ecd;       // 电机编码器值
        uint8_t state;      // 电机状态
    } fdb;

    /*设定值*/
    struct __set
    {
        float current;   // 电机电流
        float torque;    // 电机力矩
        float velocity;  // 电机转速
        float position;  // 电机位置
    } set;

} Motor_s;

/*-------------------- Motor function --------------------*/

extern void MotorInit(
    Motor_s * p_motor, uint8_t id, uint8_t can, MotorType_e motor_type, int8_t direction,
    float reduction_ratio, uint16_t mode);

#endif  // MOTOR_H
