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
#include "stm32f4xx_hal.h"

#define STDID_OFFESET 0x140

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

    CAN_SendTxMessage(&CAN_CTRL_DATA);
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

    CAN_SendTxMessage(&CAN_CTRL_DATA);
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

    CAN_SendTxMessage(&CAN_CTRL_DATA);
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

    CAN_SendTxMessage(&CAN_CTRL_DATA);
}

/**
 * @brief        多电机转矩闭环控制命令
 * @param[in]    hcan        指向CAN_HandleTypeDef结构的指针
 * @param[in]    iqControl_1 转矩电流 -2000\\~2000
 * @param[in]    iqControl_2 转矩电流 -2000\\~2000
 * @param[in]    iqControl_3 转矩电流 -2000\\~2000
 * @param[in]    iqControl_4 转矩电流 -2000\\~2000
 */
static void MultipleTorqueControl(
    hcan_t * hcan, int16_t iqControl_1, int16_t iqControl_2, int16_t iqControl_3,
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

    CAN_SendTxMessage(&CAN_CTRL_DATA);
}

/*-------------------- Check functions --------------------*/

/**
 * @brief      获取can总线句柄
 * @param[in]  motor 电机结构体
 * @return     can总线句柄
 * @note       获取电机结构体中的can号，返回对应的can总线句柄，同时检测电机类型是否为达妙电机
 */
static hcan_t * GetHcanPoint(Motor_s * motor)
{
    if (motor->type != MF_9025) return NULL;

    if (motor->can == 1)
        return &hcan1;
    else if (motor->can == 2)
        return &hcan2;

    return NULL;
}

/*-------------------- User functions --------------------*/

void LkDisableMotor(Motor_s * p_motor)
{
    hcan_t * hcan = GetHcanPoint(p_motor);
    if (hcan == NULL) return;

    DisableMotor(hcan, p_motor->id);
}

void LkStopMotor(Motor_s * p_motor)
{
    hcan_t * hcan = GetHcanPoint(p_motor);
    if (hcan == NULL) return;

    StopMotor(hcan, p_motor->id);
}

void LkEnableMotor(Motor_s * p_motor)
{
    hcan_t * hcan = GetHcanPoint(p_motor);
    if (hcan == NULL) return;

    EnableMotor(hcan, p_motor->id);
}

void LkSingleTorqueControl(Motor_s * p_motor)
{
    hcan_t * hcan = GetHcanPoint(p_motor);
    if (hcan == NULL) return;

    SingleTorqueControl(hcan, p_motor->id, p_motor->set.current);
}

void LkMultipleTorqueControl(
    Motor_s * p_motor_1, Motor_s * p_motor_2, Motor_s * p_motor_3, Motor_s * p_motor_4)
{
    hcan_t * hcan = GetHcanPoint(p_motor_1);
    if (hcan == NULL) return;

    MultipleTorqueControl(
        hcan, p_motor_1->set.current, p_motor_2->set.current, p_motor_3->set.current,
        p_motor_4->set.current);
}
/************************ END OF FILE ************************/
