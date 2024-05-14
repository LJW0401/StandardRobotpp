/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       can_cmd_dji.c/h
  * @brief      CAN发送函数，通过CAN信号控制达妙电机 8009.
  * @history
  *  Version    Date            Author          Modification
  *  V2.0.0     May-15-2024     Penguin         1. 完成。
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */
#include "CAN_cmd_damiao.h"

#include "bsp_can.h"
#include "motor.h"
#include "stm32f4xx_hal.h"
#include "struct_typedef.h"

// clang-format off
#define MIT_MODE       0x000
#define POS_MODE       0x100
#define SPEED_MODE     0x200
#define POSI_MODE      0x300

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f
// clang-format on

// 电机参数设置结构体
typedef struct __MotorCtrl
{
    int8_t mode;
    float pos_set;
    float vel_set;
    float tor_set;
    float kp_set;
    float kd_set;
} MotorCtrl_t;

struct __CanCtrlData
{
    hcan_t * hcan;
    CAN_TxHeaderTypeDef * tx_header;
    uint8_t tx_data[8];
} CAN_CTRL_DATA;

/**
************************************************************************
* @brief:      	DmEnableMotorMode: 启用电机模式函数
* @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   mode_id:  模式ID，指定要开启的模式
* @retval:     	void
* @details:    	通过CAN总线向特定电机发送启用特定模式的命令
************************************************************************
**/
static void DmEnableMotorMode(hcan_t * hcan, uint16_t motor_id, uint16_t mode_id)
{
    CAN_CTRL_DATA.hcan = hcan;

    CAN_CTRL_DATA.tx_header->StdId = motor_id + mode_id;
    CAN_CTRL_DATA.tx_header->IDE = CAN_ID_STD;
    CAN_CTRL_DATA.tx_header->RTR = CAN_RTR_DATA;
    CAN_CTRL_DATA.tx_header->DLC = 8;

    CAN_CTRL_DATA.tx_data[0] = 0xFF;
    CAN_CTRL_DATA.tx_data[1] = 0xFF;
    CAN_CTRL_DATA.tx_data[2] = 0xFF;
    CAN_CTRL_DATA.tx_data[3] = 0xFF;
    CAN_CTRL_DATA.tx_data[4] = 0xFF;
    CAN_CTRL_DATA.tx_data[5] = 0xFF;
    CAN_CTRL_DATA.tx_data[6] = 0xFF;
    CAN_CTRL_DATA.tx_data[7] = 0xFC;

    CAN_SendTxMessage(CAN_CTRL_DATA.hcan, CAN_CTRL_DATA.tx_header, CAN_CTRL_DATA.tx_data);
}

/*-------------------- User functions --------------------*/

void DmEnable(Motor_s * motor)
{
    if (motor->type != DM_8009) return;

    hcan_t * hcan = NULL;
    if (motor->can == 1)
        hcan = &hcan1;
    else if (motor->can == 2)
        hcan = &hcan2;

    if (hcan == NULL) return;

    DmEnableMotorMode(hcan, motor->id, MIT_MODE);
}

/************************ END OF FILE ************************/
