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
DJI_Motor_Send_Data_s DJI_Motor_Send_Data_CAN1_0x200 = {
    .CAN = &CAN_1,
    .std_id = DJI_200,
    .can_send_data = {0},
};
DJI_Motor_Send_Data_s DJI_Motor_Send_Data_CAN1_0x1FF = {
    .CAN = &CAN_1,
    .std_id = DJI_1FF,
    .can_send_data = {0},
};
DJI_Motor_Send_Data_s DJI_Motor_Send_Data_CAN1_0x2FF = {
    .CAN = &CAN_1,
    .std_id = DJI_2FF,
    .can_send_data = {0},
};
DJI_Motor_Send_Data_s DJI_Motor_Send_Data_CAN2_0x200 = {
    .CAN = &CAN_2,
    .std_id = DJI_200,
    .can_send_data = {0},
};
DJI_Motor_Send_Data_s DJI_Motor_Send_Data_CAN2_0x1FF = {
    .CAN = &CAN_2,
    .std_id = DJI_1FF,
    .can_send_data = {0},
};
DJI_Motor_Send_Data_s DJI_Motor_Send_Data_CAN2_0x2FF = {
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
static void CAN_SendTxMessage(CAN_HandleTypeDef *can_handle, CAN_TxHeaderTypeDef *tx_header, uint8_t *tx_data)
{
    uint32_t send_mail_box;

    uint32_t free_TxMailbox = HAL_CAN_GetTxMailboxesFreeLevel(can_handle); // 检测是否有空闲邮箱
    while (free_TxMailbox < 3)
    { // 等待空闲邮箱数达到3
        free_TxMailbox = HAL_CAN_GetTxMailboxesFreeLevel(can_handle);
    }
    HAL_CAN_AddTxMessage(can_handle, tx_header, tx_data, &send_mail_box);
}

/**
 * @brief          通过CAN控制DJI电机(支持GM3508 GM2006 GM6020)
 * @param[in]      DJI_Motor_Send_Data 电机发送数据结构体
 * @param[in]      curr_1 电机控制电流(id=1/5)
 * @param[in]      curr_2 电机控制电流(id=2/6)
 * @param[in]      curr_3 电机控制电流(id=3/7)
 * @param[in]      curr_4 电机控制电流(id=4/8)
 * @return         none
 */
void CAN_CmdDJIMotor(DJI_Motor_Send_Data_s *DJI_Motor_Send_Data, int16_t curr_1, int16_t curr_2, int16_t curr_3, int16_t curr_4)
{
    DJI_Motor_Send_Data->tx_message.StdId = DJI_Motor_Send_Data->std_id;
    DJI_Motor_Send_Data->tx_message.IDE = CAN_ID_STD;
    DJI_Motor_Send_Data->tx_message.RTR = CAN_RTR_DATA;
    DJI_Motor_Send_Data->tx_message.DLC = 0x08;
    DJI_Motor_Send_Data->can_send_data[0] = (curr_1 >> 8);
    DJI_Motor_Send_Data->can_send_data[1] = curr_1;
    DJI_Motor_Send_Data->can_send_data[2] = (curr_2 >> 8);
    DJI_Motor_Send_Data->can_send_data[3] = curr_2;
    DJI_Motor_Send_Data->can_send_data[4] = (curr_3 >> 8);
    DJI_Motor_Send_Data->can_send_data[5] = curr_3;
    DJI_Motor_Send_Data->can_send_data[6] = (curr_4 >> 8);
    DJI_Motor_Send_Data->can_send_data[7] = curr_4;
    CAN_SendTxMessage(DJI_Motor_Send_Data->CAN, &DJI_Motor_Send_Data->tx_message, DJI_Motor_Send_Data->can_send_data);
}
