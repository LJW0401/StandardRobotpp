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

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/*-------------------- DJI --------------------*/
// 发送数据
DJI_Motor_Send_Data_s DJI_MOTOR_SEND_DATA_CAN1_0X200 = {
    .CAN = &CAN_1,
    .std_id = DJI_200,
    .can_send_data = {0},
};
DJI_Motor_Send_Data_s DJI_MOTOR_SEND_DATA_CAN1_0X1FF = {
    .CAN = &CAN_1,
    .std_id = DJI_1FF,
    .can_send_data = {0},
};
DJI_Motor_Send_Data_s DJI_MOTOR_SEND_DATA_CAN1_0X2FF = {
    .CAN = &CAN_1,
    .std_id = DJI_2FF,
    .can_send_data = {0},
};
DJI_Motor_Send_Data_s DJI_MOTOR_SEND_DATA_CAN2_0X200 = {
    .CAN = &CAN_2,
    .std_id = DJI_200,
    .can_send_data = {0},
};
DJI_Motor_Send_Data_s DJI_MOTOR_SEND_DATA_CAN2_0X1FF = {
    .CAN = &CAN_2,
    .std_id = DJI_1FF,
    .can_send_data = {0},
};
DJI_Motor_Send_Data_s DJI_MOTOR_SEND_DATA_CAN2_0X2FF = {
    .CAN = &CAN_2,
    .std_id = DJI_2FF,
    .can_send_data = {0},
};

/**
 * @brief          发送控制电流
 * @param[in]      can_handle 选择CAN1或CAN2
 * @param[in]      tx_header  CAN发送数据header
 * @param[in]      tx_data    发送数据
 * @return         none
 */
static void CAN_SendTxMessage(
    CAN_HandleTypeDef * can_handle, CAN_TxHeaderTypeDef * tx_header, uint8_t * tx_data)
{
    uint32_t send_mail_box;

    uint32_t free_TxMailbox = HAL_CAN_GetTxMailboxesFreeLevel(can_handle);  // 检测是否有空闲邮箱
    while (free_TxMailbox < 3) {  // 等待空闲邮箱数达到3
        free_TxMailbox = HAL_CAN_GetTxMailboxesFreeLevel(can_handle);
    }
    HAL_CAN_AddTxMessage(can_handle, tx_header, tx_data, &send_mail_box);
}

/**
 * @brief          通过CAN控制DJI电机(支持GM3508 GM2006 GM6020)
 * @param[in]      can 发送数据使用的can口(1/2)
 * @param[in]      std_id 发送数据使用的std_id
 * @param[in]      curr_1 电机控制电流(id=1/5)
 * @param[in]      curr_2 电机控制电流(id=2/6)
 * @param[in]      curr_3 电机控制电流(id=3/7)
 * @param[in]      curr_4 电机控制电流(id=4/8)
 * @return         none
 */
void CanCmdDjiMotor(
    uint8_t can, DJI_Std_ID std_id, int16_t curr_1, int16_t curr_2, int16_t curr_3, int16_t curr_4)
{
    DJI_Motor_Send_Data_s * dji_motor_send_data = NULL;

    if (can == 1) {
        switch (std_id) {
            case DJI_200: {
                dji_motor_send_data = &DJI_MOTOR_SEND_DATA_CAN1_0X200;
            } break;
            case DJI_1FF: {
                dji_motor_send_data = &DJI_MOTOR_SEND_DATA_CAN1_0X1FF;
            } break;
            case DJI_2FF: {
                dji_motor_send_data = &DJI_MOTOR_SEND_DATA_CAN1_0X2FF;
            } break;
            case DJI_1FE: {
            } break;
            case DJI_2FE: {
            } break;
            default: {
            } break;
        }
    } else if (can == 2) {
        switch (std_id) {
            case DJI_200: {
                dji_motor_send_data = &DJI_MOTOR_SEND_DATA_CAN2_0X200;
            } break;
            case DJI_1FF: {
                dji_motor_send_data = &DJI_MOTOR_SEND_DATA_CAN2_0X1FF;
            } break;
            case DJI_2FF: {
                dji_motor_send_data = &DJI_MOTOR_SEND_DATA_CAN2_0X2FF;
            } break;
            case DJI_1FE: {
            } break;
            case DJI_2FE: {
            } break;
            default: {
            } break;
        }
    }

    if (dji_motor_send_data == NULL) return;

    dji_motor_send_data->tx_message.StdId = dji_motor_send_data->std_id;
    dji_motor_send_data->tx_message.IDE = CAN_ID_STD;
    dji_motor_send_data->tx_message.RTR = CAN_RTR_DATA;
    dji_motor_send_data->tx_message.DLC = 0x08;

    dji_motor_send_data->can_send_data[0] = (curr_1 >> 8);
    dji_motor_send_data->can_send_data[1] = curr_1;
    dji_motor_send_data->can_send_data[2] = (curr_2 >> 8);
    dji_motor_send_data->can_send_data[3] = curr_2;
    dji_motor_send_data->can_send_data[4] = (curr_3 >> 8);
    dji_motor_send_data->can_send_data[5] = curr_3;
    dji_motor_send_data->can_send_data[6] = (curr_4 >> 8);
    dji_motor_send_data->can_send_data[7] = curr_4;

    CAN_SendTxMessage(
        dji_motor_send_data->CAN, &dji_motor_send_data->tx_message,
        dji_motor_send_data->can_send_data);
}
