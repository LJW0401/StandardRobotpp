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
#include "can_cmd_cybergear.h"

#ifndef CAN_N
#define CAN_N
#define CAN_1 hcan1
#define CAN_2 hcan2
#endif

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

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

typedef enum {
    RESET_MODE = 0,        //Reset模式[复位]
    CALI_MODE = 1,         //Cali 模式[标定]
    RUN_MODE = 2           //Motor模式[运行]
} Cybergear_Mode_State_e;  //电机模式状态

typedef struct
{
    uint32_t motor_id : 8;  // 只占8位
    uint32_t data : 16;
    uint32_t mode : 5;
    uint32_t res : 3;
} __attribute__((packed)) EXT_ID_t;  // 32位扩展ID解析结构体

typedef struct
{
    uint32_t info : 24;
    uint32_t communication_type : 5;
    uint32_t res : 3;
} __attribute__((packed)) RxCAN_Info_s;  // 解码内容缓存

typedef struct
{
    uint32_t FE : 8;
    uint32_t motor_id : 16;
    uint32_t communication_type : 5;
    uint32_t res : 3;
    uint32_t MCU_id;
} __attribute__((packed)) RxCAN_Info_Type_0_s;  // 通信类型0解码内容

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
} __attribute__((packed)) RxCAN_Info_Type_2_s;  // 通信类型2解码内容

typedef struct
{
    uint32_t motor_id : 8;
    uint32_t master_can_id : 16;
    uint32_t communication_type : 5;
    uint32_t res : 3;
    uint16_t index;
} __attribute__((packed)) RxCAN_Info_Type_17_s;  // 通信类型17解码内容

typedef struct  // cybergear电机发送数据结构体
{
    CAN_HandleTypeDef * CAN;
    Cybergear_Run_Mode_e run_mode;
    EXT_ID_t EXT_ID;
    CAN_TxHeaderTypeDef tx_message;
    uint8_t txdata[8];
} Cybergrear_Send_Data_s;

/*-------------------- 变量定义 --------------------*/
static uint8_t MASTER_ID = 0x01;  //主控ID

#define CYBERGEAR_NUM 8
static Cybergrear_Send_Data_s CybergearSendData[CYBERGEAR_NUM];  //发送区与电机id对应

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
  * @brief          小米电机初始化
  * @param[out]     hmotor 电机结构体
  * @param[in]      can can1/can2
  * @param[in]      motor_id 电机id (1 ~ CYBERGEAR_NUM)
  * @retval         none
  */
void CybergearInit(CyberGear_s * hmotor, uint8_t can, uint8_t motor_id)
{
    hmotor->id = motor_id;
    if (can == 1) {
        CybergearSendData[hmotor->id - 1].CAN = &CAN_1;
    } else {
        CybergearSendData[hmotor->id - 1].CAN = &CAN_2;
    }
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
  * @param[in]      hmotor 电机结构体
  * @param[in]      torque 目标力矩
  * @param[in]      MechPosition 
  * @param[in]      velocity 
  * @param[in]      kp 
  * @param[in]      kd 
  * @retval         none
  */
void CybergearControl(
    CyberGear_s * hmotor, float torque, float MechPosition, float velocity, float kp, float kd)
{
    CybergearSendData[hmotor->id].EXT_ID.mode = 1;
    CybergearSendData[hmotor->id].EXT_ID.motor_id = hmotor->id;
    CybergearSendData[hmotor->id].EXT_ID.data = FloatToUint(torque, T_MIN, T_MAX, 16);
    CybergearSendData[hmotor->id].EXT_ID.res = 0;

    CybergearSendData[hmotor->id].txdata[0] = FloatToUint(MechPosition, P_MIN, P_MAX, 16) >> 8;
    CybergearSendData[hmotor->id].txdata[1] = FloatToUint(MechPosition, P_MIN, P_MAX, 16);
    CybergearSendData[hmotor->id].txdata[2] = FloatToUint(velocity, V_MIN, V_MAX, 16) >> 8;
    CybergearSendData[hmotor->id].txdata[3] = FloatToUint(velocity, V_MIN, V_MAX, 16);
    CybergearSendData[hmotor->id].txdata[4] = FloatToUint(kp, KP_MIN, KP_MAX, 16) >> 8;
    CybergearSendData[hmotor->id].txdata[5] = FloatToUint(kp, KP_MIN, KP_MAX, 16);
    CybergearSendData[hmotor->id].txdata[6] = FloatToUint(kd, KD_MIN, KD_MAX, 16) >> 8;
    CybergearSendData[hmotor->id].txdata[7] = FloatToUint(kd, KD_MIN, KD_MAX, 16);

    CybergearCanTx(hmotor->id);
}

/**
  * @brief          小米电机使能（通信类型 3）
  * @param[in]      hmotor 电机结构体
  * @param[in]      id 电机id
  * @retval         none
  */
void CybergearEnable(CyberGear_s * hmotor)
{
    CybergearSendData[hmotor->id].EXT_ID.mode = 3;
    CybergearSendData[hmotor->id].EXT_ID.motor_id = hmotor->id;
    CybergearSendData[hmotor->id].EXT_ID.data = MASTER_ID;
    CybergearSendData[hmotor->id].EXT_ID.res = 0;

    for (uint8_t i = 0; i < 8; i++) {
        CybergearSendData[hmotor->id].txdata[i] = 0;
    }

    CybergearCanTx(hmotor->id);
}

/**
  * @brief          电机停止运行帧（通信类型4）
  * @param[in]      hmotor 电机结构体
  * @retval         none
  */
void CybergearStop(CyberGear_s * hmotor)
{
    CybergearSendData[hmotor->id].EXT_ID.mode = 4;
    CybergearSendData[hmotor->id].EXT_ID.motor_id = hmotor->id;
    CybergearSendData[hmotor->id].EXT_ID.data = MASTER_ID;
    CybergearSendData[hmotor->id].EXT_ID.res = 0;

    for (uint8_t i = 0; i < 8; i++) {
        CybergearSendData[hmotor->id].txdata[i] = 0;
    }

    CybergearCanTx(hmotor->id);
}

/**
  * @brief          设置电机机械零位（通信类型6）会把当前电机位置设为机械零位（掉电丢失）
  * @param[in]      hmotor 电机结构体
  * @retval         none
  */
void CybergearSetMechPositionToZero(CyberGear_s * hmotor)
{
    CybergearSendData[hmotor->id].EXT_ID.mode = 6;
    CybergearSendData[hmotor->id].EXT_ID.motor_id = hmotor->id;
    CybergearSendData[hmotor->id].EXT_ID.data = MASTER_ID;
    CybergearSendData[hmotor->id].EXT_ID.res = 0;
    
    CybergearSendData[hmotor->id].txdata[0] = 1;
    for (uint8_t i = 1; i < 8; i++) {
        CybergearSendData[hmotor->id].txdata[i] = 0;
    }

    CybergearCanTx(hmotor->id);
}

/*-------------------- 封装的一些控制函数 --------------------*/

/**
  * @brief          小米电机力矩控制模式控制指令
  * @param[in]      hmotor 电机结构体
  * @param[in]      torque 目标力矩
  * @retval         none
  */
void CybergearTorqueControl(CyberGear_s* hmotor, float torque)
{
    CybergearControl(hmotor, torque, 0, 0, 0, 0);
}

/**
  * @brief          小米电机位置模式控制指令
  * @param[in]      hmotor 电机结构体
  * @param[in]      position 控制位置 (rad)
  * @param[in]      kp 响应速度(到达位置快慢)，一般取1-10
  * @param[in]      kd 电机阻尼，过小会震荡，过大电机会震动明显。一般取0.5左右
  * @retval         none
  */
void CybergearPositionControl(CyberGear_s* hmotor, float position, float kp, float kd)
{
    CybergearControl(hmotor, 0, position, 0, kp, kd);
}

/**
  * @brief          小米电机速度模式控制指令
  * @param[in]      hmotor 电机结构体
  * @param[in]      velocity 控制速度
  * @param[in]      kd 响应速度，一般取0.1-1
  * @retval         none
  */
void CybergearVelocityControl(CyberGear_s* hmotor, float velocity, float kd)
{
    CybergearControl(hmotor, 0, 0, velocity, 0, kd);
}

/************************ (C) COPYRIGHT 2024 Polarbear *****END OF FILE****/