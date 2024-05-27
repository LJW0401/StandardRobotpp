/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       can_cmd_cybergear.c/h
  * @brief      CAN发送函数，通过CAN信号控制小米电机 Cybergear.
  * @history
  *  Version    Date            Author          Modification
  *  V2.0.0     Apr-19-2024     Penguin         1. 完成。
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */
#include "CAN_cmd_cybergear.h"

#include "bsp_can.h"
#include "string.h"

#ifndef CAN_N
#define CAN_N
#define CAN_1 hcan1
#define CAN_2 hcan2
#endif

/* Private defines -----------------------------------------------------------*/
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -12.0f
#define T_MAX 12.0f

#define IQ_REF_MIN -27.0f
#define IQ_REF_MAX 27.0f
#define SPD_REF_MIN -30.0f
#define SPD_REF_MAX 30.0f
#define LIMIT_TORQUE_MIN 0.0f
#define LIMIT_TORQUE_MAX 12.0f
#define CUR_FILT_GAIN_MIN 0.0f
#define CUR_FILT_GAIN_MAX 1.0f
#define LIMIT_SPD_MIN 0.0f
#define LIMIT_SPD_MAX 30.0f
#define LIMIT_CUR_MIN 0.0f
#define LIMIT_CUR_MAX 27.0f

typedef enum {
    OK = 0,                 //无故障
    BAT_LOW_ERR = 1,        //欠压故障
    OVER_CURRENT_ERR = 2,   //过流
    OVER_TEMP_ERR = 3,      //过温
    MAGNETIC_ERR = 4,       //磁编码故障
    HALL_ERR_ERR = 5,       //HALL编码故障
    NO_CALIBRATION_ERR = 6  //未标定
} Cybergear_State_e;        //电机状态（故障信息）

typedef enum {
    CONTROL_MODE = 0,    //运控模式
    LOCATION_MODE = 1,   //位置模式
    SPEED_MODE = 2,      //速度模式
    CURRENT_MODE = 3     //电流模式
} Cybergear_Run_Mode_e;  //电机运行模式

typedef enum {
    IQ_REF = 0X7006,         //电流模式Iq指令
    SPD_REF = 0X700A,        //转速模式转速指令
    LIMIT_TORQUE = 0X700B,   //转矩限制
    CUR_KP = 0X7010,         //电流的 Kp
    CUR_KI = 0X7011,         //电流的 Ki
    CUR_FILT_GAIN = 0X7014,  //电流滤波系数filt_gain
    LOC_REF = 0X7016,        //位置模式角度指令
    LIMIT_SPD = 0X7017,      //位置模式速度设置
    LIMIT_CUR = 0X7018       //速度位置模式电流设置
} Cybergear_Index_e;         //电机功能码

typedef struct
{
    uint32_t motor_id : 8;  // 只占8位
    uint32_t data : 16;
    uint32_t mode : 5;
    uint32_t res : 3;
} __attribute__((packed)) EXT_ID_t;  // 32位扩展ID解析结构体

typedef struct  // cybergear电机发送数据结构体
{
    hcan_t * CAN;
    Cybergear_Run_Mode_e run_mode;
    EXT_ID_t EXT_ID;
    CAN_TxHeaderTypeDef tx_message;
    uint8_t txdata[8];
} Cybergear_Send_Data_s;

/*-------------------- 变量定义 --------------------*/
static uint8_t MASTER_ID = 0x01;  //主控ID

static Cybergear_Send_Data_s CybergearSendData[CYBERGEAR_NUM];  //发送区索引与电机id对应(0位不动)

/**
  * @brief          float转int，数据打包用
  * @param[in]      x float数值
  * @param[in]      x_min float数值的最小值
  * @param[in]      x_max float数值的最大值
  * @param[in]      bits  int的数据位数
  * @retval         none
  */
static uint32_t FloatToUint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max)
        x = x_max;
    else if (x < x_min)
        x = x_min;
    return (uint32_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
  * @brief          小米电机CAN通信发送
  * @param[in]      index 发送区索引
  * @retval         none
  */
static void CybergearCanTx(uint8_t index)
{
    CybergearSendData[index].tx_message.DLC = 8;
    CybergearSendData[index].tx_message.IDE = CAN_ID_EXT;
    CybergearSendData[index].tx_message.RTR = CAN_RTR_DATA;
    CybergearSendData[index].tx_message.ExtId = *((uint32_t *)&(CybergearSendData[index].EXT_ID));
    /*检测可用的发送邮箱*/
    uint32_t free_tx_mailbox =
        HAL_CAN_GetTxMailboxesFreeLevel(CybergearSendData[index].CAN);  //检测是否有空闲邮箱
    while (free_tx_mailbox < 3) {  //等待空闲邮箱数达到3
        free_tx_mailbox = HAL_CAN_GetTxMailboxesFreeLevel(CybergearSendData[index].CAN);
    }
    /* 将发送信息添加到发送邮箱中 */
    uint32_t mailbox;
    HAL_CAN_AddTxMessage(
        CybergearSendData[index].CAN,          //发送区使用的CAN总线句柄
        &CybergearSendData[index].tx_message,  //发送区的信息
        CybergearSendData[index].txdata,       //发送区的附加数据
        &mailbox);                             //将发送的数据添加到发送邮箱中
}

/*-------------------- 按照小米电机文档写的各种通信类型 --------------------*/

/**
  * @brief          运控模式电机控制指令（通信类型1）
  * @param[in]      p_motor 电机结构体
  * @param[in]      torque 目标力矩
  * @param[in]      MechPosition 
  * @param[in]      velocity 
  * @param[in]      kp 
  * @param[in]      kd 
  * @retval         none
  */
void CybergearControl(
    Motor_s * p_motor, float torque, float MechPosition, float velocity, float kp, float kd)
{
    if (p_motor->type != CYBERGEAR_MOTOR) return;

    CybergearSendData[p_motor->id].EXT_ID.mode = 1;
    CybergearSendData[p_motor->id].EXT_ID.motor_id = p_motor->id;
    CybergearSendData[p_motor->id].EXT_ID.data = FloatToUint(torque, T_MIN, T_MAX, 16);
    CybergearSendData[p_motor->id].EXT_ID.res = 0;

    CybergearSendData[p_motor->id].txdata[0] = FloatToUint(MechPosition, P_MIN, P_MAX, 16) >> 8;
    CybergearSendData[p_motor->id].txdata[1] = FloatToUint(MechPosition, P_MIN, P_MAX, 16);
    CybergearSendData[p_motor->id].txdata[2] = FloatToUint(velocity, V_MIN, V_MAX, 16) >> 8;
    CybergearSendData[p_motor->id].txdata[3] = FloatToUint(velocity, V_MIN, V_MAX, 16);
    CybergearSendData[p_motor->id].txdata[4] = FloatToUint(kp, KP_MIN, KP_MAX, 16) >> 8;
    CybergearSendData[p_motor->id].txdata[5] = FloatToUint(kp, KP_MIN, KP_MAX, 16);
    CybergearSendData[p_motor->id].txdata[6] = FloatToUint(kd, KD_MIN, KD_MAX, 16) >> 8;
    CybergearSendData[p_motor->id].txdata[7] = FloatToUint(kd, KD_MIN, KD_MAX, 16);

    if (p_motor->can == 1) {
        CybergearSendData[p_motor->id].CAN = &CAN_1;
    } else {
        CybergearSendData[p_motor->id].CAN = &CAN_2;
    }

    CybergearCanTx(p_motor->id);
}

/**
  * @brief          小米电机使能（通信类型 3）
  * @param[in]      p_motor 电机结构体
  * @retval         none
  */
void CybergearEnable(Motor_s * p_motor)
{
    if (p_motor->type != CYBERGEAR_MOTOR) return;

    CybergearSendData[p_motor->id].EXT_ID.mode = 3;
    CybergearSendData[p_motor->id].EXT_ID.motor_id = p_motor->id;
    CybergearSendData[p_motor->id].EXT_ID.data = MASTER_ID;
    CybergearSendData[p_motor->id].EXT_ID.res = 0;

    for (uint8_t i = 0; i < 8; i++) {
        CybergearSendData[p_motor->id].txdata[i] = 0;
    }

    if (p_motor->can == 1) {
        CybergearSendData[p_motor->id].CAN = &CAN_1;
    } else {
        CybergearSendData[p_motor->id].CAN = &CAN_2;
    }

    CybergearCanTx(p_motor->id);
}

/**
  * @brief          电机停止运行帧（通信类型4）
  * @param[in]      p_motor 电机结构体
  * @retval         none
  */
void CybergearStop(Motor_s * p_motor)
{
    if (p_motor->type != CYBERGEAR_MOTOR) return;

    CybergearSendData[p_motor->id].EXT_ID.mode = 4;
    CybergearSendData[p_motor->id].EXT_ID.motor_id = p_motor->id;
    CybergearSendData[p_motor->id].EXT_ID.data = MASTER_ID;
    CybergearSendData[p_motor->id].EXT_ID.res = 0;

    for (uint8_t i = 0; i < 8; i++) {
        CybergearSendData[p_motor->id].txdata[i] = 0;
    }

    if (p_motor->can == 1) {
        CybergearSendData[p_motor->id].CAN = &CAN_1;
    } else {
        CybergearSendData[p_motor->id].CAN = &CAN_2;
    }

    CybergearCanTx(p_motor->id);
}

/**
  * @brief          设置电机机械零位（通信类型6）会把当前电机位置设为机械零位（掉电丢失）
  * @param[in]      p_motor 电机结构体
  * @retval         none
  */
void CybergearSetMechPositionToZero(Motor_s * p_motor)
{
    if (p_motor->type != CYBERGEAR_MOTOR) return;

    CybergearSendData[p_motor->id].EXT_ID.mode = 6;
    CybergearSendData[p_motor->id].EXT_ID.motor_id = p_motor->id;
    CybergearSendData[p_motor->id].EXT_ID.data = MASTER_ID;
    CybergearSendData[p_motor->id].EXT_ID.res = 0;

    CybergearSendData[p_motor->id].txdata[0] = 1;
    for (uint8_t i = 1; i < 8; i++) {
        CybergearSendData[p_motor->id].txdata[i] = 0;
    }

    if (p_motor->can == 1) {
        CybergearSendData[p_motor->id].CAN = &CAN_1;
    } else {
        CybergearSendData[p_motor->id].CAN = &CAN_2;
    }

    CybergearCanTx(p_motor->id);
}

/**
  * @brief          单个参数读取（通信类型17）
  * @param[in]      p_motor 电机结构体
  * @param[in]      index 功能码
  * @retval         none
  */
void CybergearReadParam(Motor_s * p_motor, uint16_t index)
{
    if (p_motor->type != CYBERGEAR_MOTOR) return;

    CybergearSendData[p_motor->id].EXT_ID.mode = 17;
    CybergearSendData[p_motor->id].EXT_ID.motor_id = p_motor->id;
    CybergearSendData[p_motor->id].EXT_ID.data = MASTER_ID;
    CybergearSendData[p_motor->id].EXT_ID.res = 0;

    memcpy(&CybergearSendData[p_motor->id].txdata[0], &index, 2);

    for (uint8_t i = 2; i < 8; i++) {
        CybergearSendData[p_motor->id].txdata[i] = 0;
    }

    if (p_motor->can == 1) {
        CybergearSendData[p_motor->id].CAN = &CAN_1;
    } else {
        CybergearSendData[p_motor->id].CAN = &CAN_2;
    }

    CybergearCanTx(p_motor->id);
}

/*-------------------- 封装的一些控制函数 --------------------*/

/**
  * @brief          小米电机力矩控制模式控制指令
  * @param[in]      p_motor 电机结构体
  * @retval         none
  */
void CybergearTorqueControl(Motor_s * p_motor)
{
    if (p_motor->type != CYBERGEAR_MOTOR) return;

    CybergearControl(p_motor, p_motor->set.tor, 0, 0, 0, 0);
}

/**
  * @brief          小米电机位置模式控制指令
  * @param[in]      p_motor 电机结构体
  * @param[in]      kp 响应速度(到达位置快慢)，一般取1-10
  * @param[in]      kd 电机阻尼，过小会震荡，过大电机会震动明显。一般取0.5左右
  * @retval         none
  */
void CybergearPositionControl(Motor_s * p_motor, float kp, float kd)
{
    if (p_motor->type != CYBERGEAR_MOTOR) return;

    CybergearControl(p_motor, 0, p_motor->set.pos, 0, kp, kd);
}

/**
  * @brief          小米电机速度模式控制指令
  * @param[in]      p_motor 电机结构体
  * @param[in]      kd 响应速度，一般取0.1-1
  * @retval         none
  */
void CybergearVelocityControl(Motor_s * p_motor, float kd)
{
    if (p_motor->type != CYBERGEAR_MOTOR) return;

    CybergearControl(p_motor, 0, 0, p_motor->set.vel, 0, kd);
}

/************************ END OF FILE ************************/
