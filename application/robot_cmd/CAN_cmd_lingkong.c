/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       can_cmd_lingkong.c/h
  * @brief      CAN发送函数，通过CAN信号控制瓴控电机 9025.
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     May-16-2024     Penguin         1. 完成。
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */
#include "CAN_cmd_lingkong.h"

#include "bsp_can.h"
#include "motor.h"
#include "stm32f4xx_hal.h"

#define STDID_OFFESET 0x140

typedef struct __CanCtrlData
{
    hcan_t * hcan;
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
} CanCtrlData_s;

static CanCtrlData_s CAN_CTRL_DATA = {
    .tx_header.IDE = CAN_ID_STD,
    .tx_header.RTR = CAN_RTR_DATA,
    .tx_header.DLC = 8,
};

/*-------------------- Private functions --------------------*/

/**
 * @brief        电机失能
 * @param[in]    hcan     指向CAN_HandleTypeDef结构的指针
 * @param[in]    motor_id 电机ID，指定目标电机
 */
static void DisableMotor(hcan_t * hcan, uint16_t motor_id)
{
    CAN_CTRL_DATA.hcan = hcan;

    CAN_CTRL_DATA.tx_header.StdId = motor_id + STDID_OFFESET;

    CAN_CTRL_DATA.tx_data[0] = 0x80;
    CAN_CTRL_DATA.tx_data[1] = 0x00;
    CAN_CTRL_DATA.tx_data[2] = 0x00;
    CAN_CTRL_DATA.tx_data[3] = 0x00;
    CAN_CTRL_DATA.tx_data[4] = 0x00;
    CAN_CTRL_DATA.tx_data[5] = 0x00;
    CAN_CTRL_DATA.tx_data[6] = 0x00;
    CAN_CTRL_DATA.tx_data[7] = 0x00;

    CAN_SendTxMessage(CAN_CTRL_DATA.hcan, &CAN_CTRL_DATA.tx_header, CAN_CTRL_DATA.tx_data);
}

/**
 * @brief        停止电机
 * @param[in]    hcan     指向CAN_HandleTypeDef结构的指针
 * @param[in]    motor_id 电机ID，指定目标电机
 */
static void StopMotor(hcan_t * hcan, uint16_t motor_id)
{
    CAN_CTRL_DATA.hcan = hcan;

    CAN_CTRL_DATA.tx_header.StdId = motor_id + STDID_OFFESET;

    CAN_CTRL_DATA.tx_data[0] = 0x81;
    CAN_CTRL_DATA.tx_data[1] = 0x00;
    CAN_CTRL_DATA.tx_data[2] = 0x00;
    CAN_CTRL_DATA.tx_data[3] = 0x00;
    CAN_CTRL_DATA.tx_data[4] = 0x00;
    CAN_CTRL_DATA.tx_data[5] = 0x00;
    CAN_CTRL_DATA.tx_data[6] = 0x00;
    CAN_CTRL_DATA.tx_data[7] = 0x00;

    CAN_SendTxMessage(CAN_CTRL_DATA.hcan, &CAN_CTRL_DATA.tx_header, CAN_CTRL_DATA.tx_data);
}

/**
 * @brief        电机使能
 * @param[in]    hcan     指向CAN_HandleTypeDef结构的指针
 * @param[in]    motor_id 电机ID，指定目标电机
 */
static void EnableMotor(hcan_t * hcan, uint16_t motor_id)
{
    CAN_CTRL_DATA.hcan = hcan;

    CAN_CTRL_DATA.tx_header.StdId = motor_id + STDID_OFFESET;

    CAN_CTRL_DATA.tx_data[0] = 0x88;
    CAN_CTRL_DATA.tx_data[1] = 0x00;
    CAN_CTRL_DATA.tx_data[2] = 0x00;
    CAN_CTRL_DATA.tx_data[3] = 0x00;
    CAN_CTRL_DATA.tx_data[4] = 0x00;
    CAN_CTRL_DATA.tx_data[5] = 0x00;
    CAN_CTRL_DATA.tx_data[6] = 0x00;
    CAN_CTRL_DATA.tx_data[7] = 0x00;

    CAN_SendTxMessage(CAN_CTRL_DATA.hcan, &CAN_CTRL_DATA.tx_header, CAN_CTRL_DATA.tx_data);
}

/**
 * @brief        单电机转矩闭环控制命令
 * @param[in]    hcan      指向CAN_HandleTypeDef结构的指针
 * @param[in]    motor_id  电机ID，指定目标电机
 * @param[in]    iqControl 转矩电流 -2048~2048
 */
static void SingleTorqueControl(hcan_t * hcan, uint16_t motor_id, int16_t iqControl)
{
    CAN_CTRL_DATA.hcan = hcan;

    CAN_CTRL_DATA.tx_header.StdId = motor_id + STDID_OFFESET;

    CAN_CTRL_DATA.tx_data[0] = 0xA1;
    CAN_CTRL_DATA.tx_data[1] = 0x00;
    CAN_CTRL_DATA.tx_data[2] = 0x00;
    CAN_CTRL_DATA.tx_data[3] = 0x00;
    CAN_CTRL_DATA.tx_data[4] = *(uint8_t *)(&iqControl);
    CAN_CTRL_DATA.tx_data[5] = *((uint8_t *)(&iqControl) + 1);
    CAN_CTRL_DATA.tx_data[6] = 0x00;
    CAN_CTRL_DATA.tx_data[7] = 0x00;

    CAN_SendTxMessage(CAN_CTRL_DATA.hcan, &CAN_CTRL_DATA.tx_header, CAN_CTRL_DATA.tx_data);
}

/**
 * @brief        多电机转矩闭环控制命令
 * @param[in]    hcan        指向CAN_HandleTypeDef结构的指针
 * @param[in]    motor_id    电机ID，指定目标电机
 * @param[in]    iqControl_1 转矩电流 -2000~2000
 * @param[in]    iqControl_2 转矩电流 -2000~2000
 * @param[in]    iqControl_3 转矩电流 -2000~2000
 * @param[in]    iqControl_4 转矩电流 -2000~2000
 */
static void MultipleTorqueControl(
    hcan_t * hcan, uint16_t motor_id, int16_t iqControl_1, int16_t iqControl_2, int16_t iqControl_3,
    int16_t iqControl_4)
{
    CAN_CTRL_DATA.hcan = hcan;

    CAN_CTRL_DATA.tx_header.StdId = 0x280;

    CAN_CTRL_DATA.tx_data[0] = *(uint8_t *)(&iqControl_1);
    CAN_CTRL_DATA.tx_data[1] = *((uint8_t *)(&iqControl_1) + 1);
    CAN_CTRL_DATA.tx_data[2] = *(uint8_t *)(&iqControl_1);
    CAN_CTRL_DATA.tx_data[3] = *((uint8_t *)(&iqControl_1) + 1);
    CAN_CTRL_DATA.tx_data[4] = *(uint8_t *)(&iqControl_3);
    CAN_CTRL_DATA.tx_data[5] = *((uint8_t *)(&iqControl_3) + 1);
    CAN_CTRL_DATA.tx_data[6] = *(uint8_t *)(&iqControl_4);
    CAN_CTRL_DATA.tx_data[7] = *((uint8_t *)(&iqControl_4) + 1);

    CAN_SendTxMessage(CAN_CTRL_DATA.hcan, &CAN_CTRL_DATA.tx_header, CAN_CTRL_DATA.tx_data);
}

/*-------------------- User functions --------------------*/

/************************ END OF FILE ************************/
