/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       can_cmd_dji.c/h
  * @brief      CAN发送函数，通过CAN信号控制DJI电机 GM3508 GM2006 GM6020.
  * @history
  *  Version    Date            Author          Modification
  *  V2.0.0     Mar-27-2024     Penguin         1. 完成。
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#include "CAN_cmd_dji.h"

/*-------------------- Global var --------------------*/

static CanCtrlData_s CAN_CTRL_DATA = {
    .tx_header.IDE = CAN_ID_STD,
    .tx_header.RTR = CAN_RTR_DATA,
    .tx_header.DLC = 8,
};

/*-------------------- Private functions --------------------*/

/**
 * @brief          通过CAN发送控制电流控制DJI电机(支持GM3508 GM2006 GM6020)
 * @param[in]      can 发送数据使用的can口
 * @param[in]      std_id 发送数据使用的std_id
 * @param[in]      curr_1 电机控制电流
 * @param[in]      curr_2 电机控制电流
 * @param[in]      curr_3 电机控制电流
 * @param[in]      curr_4 电机控制电流
 * @return         none
 */
static void MultipleCurrentControl(
    hcan_t * hcan, uint16_t std_id, int16_t curr_1, int16_t curr_2, int16_t curr_3, int16_t curr_4)
{
    CAN_CTRL_DATA.hcan = hcan;

    CAN_CTRL_DATA.tx_header.StdId = std_id;

    CAN_CTRL_DATA.tx_data[0] = (curr_1 >> 8);
    CAN_CTRL_DATA.tx_data[1] = curr_1;
    CAN_CTRL_DATA.tx_data[2] = (curr_2 >> 8);
    CAN_CTRL_DATA.tx_data[3] = curr_2;
    CAN_CTRL_DATA.tx_data[4] = (curr_3 >> 8);
    CAN_CTRL_DATA.tx_data[5] = curr_3;
    CAN_CTRL_DATA.tx_data[6] = (curr_4 >> 8);
    CAN_CTRL_DATA.tx_data[7] = curr_4;

    CAN_SendTxMessage(&CAN_CTRL_DATA);
}

/**
 * @brief          通过CAN发送控制电压控制DJI电机(支持GM3508 GM2006 GM6020)
 * @param[in]      can 发送数据使用的can口
 * @param[in]      std_id 发送数据使用的std_id
 * @param[in]      volt_1 电机控制电压
 * @param[in]      volt_2 电机控制电压
 * @param[in]      volt_3 电机控制电压
 * @param[in]      volt_4 电机控制电压
 * @return         none
 */
static void MultipleVoltageControl(
    hcan_t * hcan, uint16_t std_id, int16_t volt_1, int16_t volt_2, int16_t volt_3, int16_t volt_4)
{
    CAN_CTRL_DATA.hcan = hcan;

    CAN_CTRL_DATA.tx_header.StdId = std_id;

    CAN_CTRL_DATA.tx_data[0] = (volt_1 >> 8);
    CAN_CTRL_DATA.tx_data[1] = volt_1;
    CAN_CTRL_DATA.tx_data[2] = (volt_2 >> 8);
    CAN_CTRL_DATA.tx_data[3] = volt_2;
    CAN_CTRL_DATA.tx_data[4] = (volt_3 >> 8);
    CAN_CTRL_DATA.tx_data[5] = volt_3;
    CAN_CTRL_DATA.tx_data[6] = (volt_4 >> 8);
    CAN_CTRL_DATA.tx_data[7] = volt_4;

    CAN_SendTxMessage(&CAN_CTRL_DATA);
}
/*-------------------- User function --------------------*/

/**
 * @brief          通过CAN控制DJI电机(支持GM3508 GM2006 GM6020)
 * @param[in]      can 发送数据使用的can口(1/2)
 * @param[in]      std_id 发送数据使用的std_id
 * @param[in]      curr_1 电机控制电流(id=1/5)
 * @param[in]      curr_2 电机控制电流(id=2/6)
 * @param[in]      curr_3 电机控制电流(id=3/7)
 * @param[in]      curr_4 电机控制电流(id=4/8)
 * @return         none
 * @note           老的控制方式的兼容函数，等后期的安全函数上线后会删除
 */
void CanCmdDjiMotor(
    uint8_t can, uint16_t std_id, int16_t curr_1, int16_t curr_2, int16_t curr_3, int16_t curr_4)
{
    hcan_t * hcan = NULL;
    if (can == 1)
        hcan = &hcan1;
    else if (can == 2)
        hcan = &hcan2;
    if (hcan == NULL) return;

    CAN_CTRL_DATA.hcan = hcan;

    CAN_CTRL_DATA.tx_header.StdId = std_id;

    CAN_CTRL_DATA.tx_data[0] = (curr_1 >> 8);
    CAN_CTRL_DATA.tx_data[1] = curr_1;
    CAN_CTRL_DATA.tx_data[2] = (curr_2 >> 8);
    CAN_CTRL_DATA.tx_data[3] = curr_2;
    CAN_CTRL_DATA.tx_data[4] = (curr_3 >> 8);
    CAN_CTRL_DATA.tx_data[5] = curr_3;
    CAN_CTRL_DATA.tx_data[6] = (curr_4 >> 8);
    CAN_CTRL_DATA.tx_data[7] = curr_4;

    CAN_SendTxMessage(&CAN_CTRL_DATA);
}

/**
 * @brief 暂时还不可使用！！！ dji多电机电流控制
 * @param p_motor_1 电机1
 * @param p_motor_2 电机2
 * @param p_motor_3 电机3
 * @param p_motor_4 电机4
 */
void DjiMultipleControl(
    Motor_s * p_motor_1, Motor_s * p_motor_2, Motor_s * p_motor_3, Motor_s * p_motor_4)
{
    hcan_t * hcan = NULL;
    if (p_motor_1->can == 1)
        hcan = &hcan1;
    else if (p_motor_1->can == 2)
        hcan = &hcan2;
    if (hcan == NULL) return;

    uint16_t curr[4] = {0, 0, 0, 0};

    MultipleCurrentControl(hcan, p_motor_1->mode, curr[0], curr[1], curr[2], curr[3]);
    MultipleVoltageControl(hcan, p_motor_1->mode, 0, 0, 0, 0);
}

/************************ END OF FILE ************************/
