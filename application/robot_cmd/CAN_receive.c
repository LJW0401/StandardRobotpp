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

#include <string.h>

#include "bsp_can.h"
#include "cmsis_os.h"
#include "detect_task.h"

// motor data read
#define get_dji_motor_measure(ptr, data)                               \
    {                                                                  \
        (ptr)->last_ecd = (ptr)->ecd;                                  \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
        (ptr)->temperate = (data)[6];                                  \
    }

// 接收数据
static DjiMotorMeasure_t CAN1_DJI_MEASURE[11];
static DjiMotorMeasure_t CAN2_DJI_MEASURE[11];

static CybergearMeasure_s CAN1_CYBERGEAR_MEASURE[CYBERGEAR_NUM + 1];
static CybergearMeasure_s CAN2_CYBERGEAR_MEASURE[CYBERGEAR_NUM + 1];

static DmMeasure_s CAN1_DM_MEASURE[DM_NUM];
static DmMeasure_s CAN2_DM_MEASURE[DM_NUM];

/*-------------------- Decode --------------------*/

/**
************************************************************************
* @brief:      	DmFdbData: 获取DM电机反馈数据函数
* @param[in]:   motor:    指向motor_t结构的指针，包含电机相关信息和反馈数据
* @param[in]:   rx_data:  指向包含反馈数据的数组指针
* @retval:     	void
* @details:    	从接收到的数据中提取DM4310电机的反馈信息，包括电机ID、
*               状态、位置、速度、扭矩以及相关温度参数
************************************************************************
**/
void DmFdbData(DmMeasure_s * dm_measure, uint8_t * rx_data)
{
    dm_measure->id = (rx_data[0]) & 0x0F;
    dm_measure->err = (rx_data[0]) >> 4;
    dm_measure->p_int = (rx_data[1] << 8) | rx_data[2];
    dm_measure->v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
    dm_measure->t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
    dm_measure->pos = uint_to_float(dm_measure->p_int, -12.5, 12.5, 16);  // (-12.5,12.5)
    dm_measure->vel = uint_to_float(dm_measure->v_int, -45.0, 45.0, 12);  // (-45.0,45.0)
    dm_measure->tor = uint_to_float(dm_measure->t_int, -18.0, 18.0, 12);  // (-18.0,18.0)
    dm_measure->t_mos = (float)(rx_data[6]);
    dm_measure->t_rotor = (float)(rx_data[7]);
}

/**
 * @brief          若接收到的数据标识符为StdId则对应解码
 * @note           解码数据包括DJI电机数据与板间通信数据
 * @param[in]      CAN CAN口(CAN_1或CAN_2)
 * @param[in]      rx_header CAN接收数据头
 * @param[in]      rx_data CAN接收数据
 */
static void DecodeStdIdData(hcan_t * CAN, CAN_RxHeaderTypeDef * rx_header, uint8_t rx_data[8])
{
    switch (rx_header->StdId) {
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
        case DJI_M11_ID: {  // 以上ID为DJI电机标识符
            static uint8_t i = 0;
            i = rx_header->StdId - DJI_M1_ID;
            if (CAN == &hcan1)  // 接收到的数据是通过 CAN1 接收的
            {
                get_dji_motor_measure(&CAN1_DJI_MEASURE[i], rx_data);
            } else if (CAN == &hcan2)  // 接收到的数据是通过 CAN2 接收的
            {
                get_dji_motor_measure(&CAN2_DJI_MEASURE[i], rx_data);
            }
            break;
        }
        case DM_M1_ID:
        case DM_M2_ID:
        case DM_M3_ID:
        case DM_M4_ID:
        case DM_M5_ID:
        case DM_M6_ID: {  // 以上ID为DM电机标识符
            static uint8_t i = 0;
            i = rx_header->StdId - DM_M1_ID;
            if (CAN == &hcan1)  // 接收到的数据是通过 CAN1 接收的
            {
                DmFdbData(&CAN1_DM_MEASURE[i], rx_data);
            } else if (CAN == &hcan2)  // 接收到的数据是通过 CAN2 接收的
            {
                DmFdbData(&CAN2_DM_MEASURE[i], rx_data);
            }
        } break;
        default: {
            break;
        }
    }
}

/**
  * @brief          小米电机反馈帧解码（通信类型2）
  * @param[in]      p_motor 电机结构体
  * @param[in]      rx_data[8] CAN线接收到的数据
  * @note           将接收到的CAN线数据解码到电机结构体中
  * @retval         none
  */
static void CybergearRxDecode(Motor_s * p_motor, uint8_t rx_data[8])
{
    uint16_t decode_temp_mi;  //小米电机反馈数据解码缓冲
    decode_temp_mi = (rx_data[0] << 8 | rx_data[1]);
    p_motor->fdb.pos = ((float)decode_temp_mi - 32767.5f) / 32767.5f * 4 * 3.1415926f;

    decode_temp_mi = (rx_data[2] << 8 | rx_data[3]);
    p_motor->fdb.w = ((float)decode_temp_mi - 32767.5f) / 32767.5f * 30.0f;

    decode_temp_mi = (rx_data[4] << 8 | rx_data[5]);
    p_motor->fdb.T = ((float)decode_temp_mi - 32767.5f) / 32767.5f * 12.0f;

    decode_temp_mi = (rx_data[6] << 8 | rx_data[7]);
    p_motor->fdb.temperature = (float)decode_temp_mi / 10.0f;
}

/**
 * @brief          若接收到的数据标识符为ExtId则对应解码
 * @note           解码数据包括cybergear电机数据与板间通信数据
 * @param[in]      CAN CAN口(CAN_1或CAN_2)
 * @param[in]      rx_header CAN接收数据头
 * @param[in]      rx_data CAN接收数据
 */
static void DecodeExtIdData(hcan_t * CAN, CAN_RxHeaderTypeDef * rx_header, uint8_t rx_data[8])
{
    uint8_t motor_id = 0;
    if (((RxCanInfo_s *)(&rx_header->ExtId))->communication_type == 2) {  //通信类型2
        motor_id = ((RxCanInfoType_2_s *)(&rx_header->ExtId))->motor_id;
    }

    if (CAN == &hcan1)  // 接收到的数据是通过 CAN1 接收的
    {
        memcpy(&CAN1_CYBERGEAR_MEASURE[motor_id].ext_id, &rx_header->ExtId, 4);
        memcpy(CAN1_CYBERGEAR_MEASURE[motor_id].rx_data, rx_data, 8);
    } else if (CAN == &hcan2)  // 接收到的数据是通过 CAN2 接收的
    {
        memcpy(&CAN2_CYBERGEAR_MEASURE[motor_id].ext_id, &rx_header->ExtId, 4);
        memcpy(CAN2_CYBERGEAR_MEASURE[motor_id].rx_data, rx_data, 8);
    }
}

/*-------------------- Callback --------------------*/

/**
 * @brief          hal库CAN回调函数,接收电机数据
 * @param[in]      hcan:CAN句柄指针
 * @retval         none
 */
void HAL_CAN_RxFifo0MsgPendingCallback(hcan_t * hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if (rx_header.IDE == CAN_ID_STD)  // 接收到的数据标识符为StdId
    {
        DecodeStdIdData(hcan, &rx_header, rx_data);
    } else if (rx_header.IDE == CAN_ID_EXT)  // 接收到的数据标识符为ExtId
    {
        DecodeExtIdData(hcan, &rx_header, rx_data);
    }
}

/*-------------------- Get data --------------------*/

/**
 * @brief          获取DJI电机接收数据指针
 * @param[in]      can can口 (1 or 2)
 * @param[in]      i 电机接收数据索引,范围[0,11]
 * @return         DJI_Motor_Measure_Data
 * @note           如果输入值超出范围则返回CAN1_DJI_motor[1]
 */
const DjiMotorMeasure_t * GetDjiMotorMeasurePoint(uint8_t can, uint8_t i)
{
    if (i < 12) {
        if (can == 1) {
            return &CAN1_DJI_MEASURE[i];
        } else if (can == 2) {
            return &CAN2_DJI_MEASURE[i];
        }
    }
    return &CAN1_DJI_MEASURE[1];
}

CybergearModeState_e GetCybergearModeState(Motor_s * p_motor)
{
    if (p_motor->type != CYBERGEAR_MOTOR) return UNDEFINED_MODE;

    // clang-format off
    if (p_motor->can == 1) {
        return (CybergearModeState_e)(((RxCanInfoType_2_s *)(&CAN1_CYBERGEAR_MEASURE[p_motor->id].ext_id))->mode_state);
    } else {
        return (CybergearModeState_e)(((RxCanInfoType_2_s *)(&CAN2_CYBERGEAR_MEASURE[p_motor->id].ext_id))->mode_state);
    }
    // clang-format on
}

/**
 * @brief          获取接收数据
 * @param[out]     p_motor 电机结构体
 * @return         none
 */
void GetMotorMeasure(Motor_s * p_motor)
{
    switch (p_motor->type) {
        case DJI_M2006:
        case DJI_M3508: {
            const DjiMotorMeasure_t * p_dji_motor_measure =
                GetDjiMotorMeasurePoint(p_motor->can, p_motor->id - 1);
            p_motor->fdb.w = p_dji_motor_measure->speed_rpm * RPM_TO_OMEGA *
                             p_motor->reduction_ratio * p_motor->direction;
            p_motor->fdb.pos = p_dji_motor_measure->ecd * 2 * M_PI / 8192 - M_PI;
            p_motor->fdb.temperature = p_dji_motor_measure->temperate;
            p_motor->fdb.current = p_dji_motor_measure->given_current;
            p_motor->fdb.ecd = p_dji_motor_measure->ecd;
        } break;
        case DJI_M6020: {
            const DjiMotorMeasure_t * p_dji_motor_measure =
                GetDjiMotorMeasurePoint(p_motor->can, p_motor->id + 3);
            p_motor->fdb.w = p_dji_motor_measure->speed_rpm * RPM_TO_OMEGA *
                             p_motor->reduction_ratio * p_motor->direction;
            p_motor->fdb.pos = p_dji_motor_measure->ecd * 2 * M_PI / 8192 - M_PI;
            p_motor->fdb.temperature = p_dji_motor_measure->temperate;
            p_motor->fdb.current = p_dji_motor_measure->given_current;
            p_motor->fdb.ecd = p_dji_motor_measure->ecd;
        } break;
        case CYBERGEAR_MOTOR: {
            if (p_motor->can == 1) {
                CybergearRxDecode(p_motor, CAN1_CYBERGEAR_MEASURE[p_motor->id].rx_data);
            } else {
                CybergearRxDecode(p_motor, CAN2_CYBERGEAR_MEASURE[p_motor->id].rx_data);
            }
        } break;
        case DM_8009: {
        } break;
        case MF_9025: {
        } break;
        default:
            break;
    }
}
