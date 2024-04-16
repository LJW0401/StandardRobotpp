/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       can_receive.c/h
  * @brief      CAN中断接收函数，接收电机数据.
  * @note       支持DJI电机 GM3508 GM2006 GM6020
  *         未来支持小米电机 Cybergear
  *         未来支持达妙电机 DM8009
  * @history
  *  Version    Date            Author          Modification
  *  V2.0.0     Mar-27-2024     Penguin         1. 添加CAN发送函数和新的电机控制函数，解码中将CAN1 CAN2分开。
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#include "CAN_receive.h"

#include "cmsis_os.h"

#include "detect_task.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
// motor data read
#define get_motor_measure(ptr, data)                                   \
    {                                                                  \
        (ptr)->last_ecd = (ptr)->ecd;                                  \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
        (ptr)->temperate = (data)[6];                                  \
    }

// 接收数据
static DJI_Motor_Measure_t CAN1_DJI_motor[11];
static DJI_Motor_Measure_t CAN2_DJI_motor[11];

/**
 * @brief          若接收到的数据标识符为StdId则对应解码
 * @note           解码数据包括DJI电机数据与板间通信数据
 * @param[in]      CAN CAN口(CAN_1或CAN_2)
 * @param[in]      rx_header CAN接收数据头
 * @param[in]      rx_data CAN接收数据
 */
void DecodeStdIdData(CAN_HandleTypeDef *CAN, CAN_RxHeaderTypeDef *rx_header, uint8_t rx_data[8])
{
    switch (rx_header->StdId)
    {
    case DJI_M1_ID:
    case DJI_M2_ID:
    case DJI_M3_ID:
    case DJI_M4_ID:
    case DJI_M5_ID:
    case DJI_M6_ID:
    case DJI_M7_ID:
    case DJI_M8_ID:
    case DJI_M9_ID:
    case DJI_M10_ID:
    case DJI_M11_ID:
    { // 以上ID为DJI电机标识符
        static uint8_t i = 0;
        i = rx_header->StdId - DJI_M1_ID;
        if (CAN == &hcan1) // 接收到的数据是通过 CAN1 接收的
        {
            get_motor_measure(&CAN1_DJI_motor[i], rx_data);
        }
        else if (CAN == &hcan2) // 接收到的数据是通过 CAN2 接收的
        {
            get_motor_measure(&CAN2_DJI_motor[i], rx_data);
        }
        break;
    }
    default:
    {
        break;
    }
    }
}

/**
 * @brief          若接收到的数据标识符为ExtId则对应解码
 * @note           解码数据包括...
 * @param[in]      CAN CAN口(CAN_1或CAN_2)
 * @param[in]      rx_header CAN接收数据头
 * @param[in]      rx_data CAN接收数据
 */
void DecodeExtIdData(CAN_HandleTypeDef *CAN, CAN_RxHeaderTypeDef *rx_header, uint8_t rx_data[8])
{
    /*
    完成解码内容
    */
}

/**
 * @brief          hal库CAN回调函数,接收电机数据
 * @param[in]      hcan:CAN句柄指针
 * @retval         none
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if (rx_header.IDE == CAN_ID_STD) // 接收到的数据标识符为StdId
    {
        DecodeStdIdData(hcan, &rx_header, rx_data);
    }
    else if (rx_header.IDE == CAN_ID_EXT) // 接收到的数据标识符为ExtId
    {
        DecodeExtIdData(hcan, &rx_header, rx_data);
    }
}

/**
 * @brief          获取DJI电机接收数据指针
 * @param[in]      can can口 (1 or 2)
 * @param[in]      i 电机编号,范围[0,11]
 * @return         DJI_Motor_Measure_Data
 * @note           如果输入值超出范围则返回CAN1_DJI_motor[1]
 */
const DJI_Motor_Measure_t *GetDjiMotorMeasurePoint(uint8_t can, uint8_t i)
{
    if (i < 12)
    {
        if (can == 1)
        {
            return &CAN1_DJI_motor[i];
        }
        else if (can == 2)
        {
            return &CAN2_DJI_motor[i];
        }
    }
    return &CAN1_DJI_motor[1];
}
