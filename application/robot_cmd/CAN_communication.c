/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       CAN_communication.c/h
  * @brief      CAN通信部分
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     May-27-2024     Penguin         1. done
  *
  @verbatim
  ==============================================================================
板间通信时stdid的内容如下
data_type + (data_id << 4) + target_id

bit 0-3: target_id
bit 4-7: data_id
bit 8-11: data_type

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */
#include "CAN_communication.h"

#include "bsp_can.h"

static CanCtrlData_s CAN_CTRL_DATA = {
    .tx_header.IDE = CAN_ID_STD,
    .tx_header.RTR = CAN_RTR_DATA,
    .tx_header.DLC = 8,
};

/*-------------------- Private functions --------------------*/
// 板间通信

/**
 * @brief          发送自定义数据
 * @param[in]      hcan CAN句柄
 * @param[in]      data_id 数据包ID
 * @param[in]      target_id 目标板ID
 * @param[in]      data 包含8个字节的数据的指针
 * @retval         none
 */
static void SendData(hcan_t * hcan, uint16_t data_id, uint16_t target_id, uint8_t * data)
{
    CAN_CTRL_DATA.hcan = hcan;

    CAN_CTRL_DATA.tx_header.StdId = BOARD_DATA_ANY + (data_id << 4) + target_id;

    for (size_t i = 0; i < 8; i++) {
        CAN_CTRL_DATA.tx_data[i] = data[i];
    }

    CAN_SendTxMessage(&CAN_CTRL_DATA);
}

/**
 * @brief          发送uint16数据
 * @param[in]      hcan CAN句柄
 * @param[in]      data_id 数据包ID
 * @param[in]      target_id 目标板ID
 * @param[in]      data_1 数据1
 * @param[in]      data_2 数据2
 * @param[in]      data_3 数据3
 * @param[in]      data_4 数据4
 * @retval         none
 */
static void Uint16SendData(
    hcan_t * hcan, uint16_t data_id, uint16_t target_id, uint16_t data_1, uint16_t data_2,
    uint16_t data_3, uint16_t data_4)
{
    CAN_CTRL_DATA.hcan = hcan;

    CAN_CTRL_DATA.tx_header.StdId = BOARD_DATA_UINT16 + (data_id << 4) + target_id;

    CAN_CTRL_DATA.tx_data[0] = data_1 >> 8;
    CAN_CTRL_DATA.tx_data[1] = data_1;
    CAN_CTRL_DATA.tx_data[2] = data_2 >> 8;
    CAN_CTRL_DATA.tx_data[3] = data_2;
    CAN_CTRL_DATA.tx_data[4] = data_3 >> 8;
    CAN_CTRL_DATA.tx_data[5] = data_3;
    CAN_CTRL_DATA.tx_data[6] = data_4 >> 8;
    CAN_CTRL_DATA.tx_data[7] = data_4;

    CAN_SendTxMessage(&CAN_CTRL_DATA);
}

/*-------------------- Public functions --------------------*/

/**
 * @brief          CAN发送数据到目标板
 * @param[in]      can can口
 * @param[in]      data_id 数据包ID
 * @param[in]      target_id 目标板ID
 * @param[in]      data  包含8个字节的数据的指针
 * @retval         none
 */
void CanSendDataToBoard(uint8_t can, uint16_t data_id, uint16_t target_id, uint8_t * data)
{
    if (can == 1)
        SendData(&hcan1, data_id, target_id, data);
    else if (can == 2)
        SendData(&hcan2, data_id, target_id, data);
}

/**
 * @brief          CAN发送uint16数据到目标板
 * @param[in]      can can口
 * @param[in]      data_id 数据包ID
 * @param[in]      target_id 目标板ID
 * @param[in]      data_1 数据1
 * @param[in]      data_2 数据2
 * @param[in]      data_3 数据3
 * @param[in]      data_4 数据4
 * @retval         none
 */
void CanSendUint16DataToBoard(
    uint8_t can, uint16_t data_id, uint16_t target_id, uint16_t data_1, uint16_t data_2,
    uint16_t data_3, uint16_t data_4)
{
    if (can == 1)
        Uint16SendData(&hcan1, data_id, target_id, data_1, data_2, data_3, data_4);
    else if (can == 2)
        Uint16SendData(&hcan2, data_id, target_id, data_1, data_2, data_3, data_4);
}