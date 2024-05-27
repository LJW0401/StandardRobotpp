/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       can_receive.c/h
  * @brief      CAN中断接收函数，接收电机数据.
  * @note       支持DJI电机 GM3508 GM2006 GM6020
  *             支持小米电机 Cybergear
  *             支持达妙电机 DM8009
  *             支持瓴控电机 MF9025
  * @history
  *  Version    Date            Author          Modification
  *  V2.0.0     Mar-27-2024     Penguin         1. 添加CAN发送函数和新的电机控制函数，解码中将CAN1 CAN2分开。
  *  V2.1.0     Mar-20-2024     Penguin         1. 添加DM电机的适配
  *  V2.2.0     May-22-2024     Penguin         1. 添加LK电机的适配
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#include "CAN_receive.h"

#include "bsp_can.h"
#include "cmsis_os.h"
#include "detect_task.h"
#include "robot_param.h"
#include "string.h"
#include "usb_task.h"
#include "user_lib.h"

#define DATA_NUM 10

// 接收数据
static DjiMotorMeasure_t CAN1_DJI_MEASURE[11];
static DjiMotorMeasure_t CAN2_DJI_MEASURE[11];

static CybergearMeasure_s CAN1_CYBERGEAR_MEASURE[CYBERGEAR_NUM + 1];
static CybergearMeasure_s CAN2_CYBERGEAR_MEASURE[CYBERGEAR_NUM + 1];

static DmMeasure_s CAN1_DM_MEASURE[DM_NUM];
static DmMeasure_s CAN2_DM_MEASURE[DM_NUM];

static LkMeasure_s CAN1_LK_MEASURE[LK_NUM];
static LkMeasure_s CAN2_LK_MEASURE[LK_NUM];

static uint8_t OTHER_BOARD_DATA_ANY[DATA_NUM][8];
static uint16_t OTHER_BOARD_DATA_UINT16[DATA_NUM][4];

/*-------------------- Decode --------------------*/

/**
 * @brief        DmFdbData: 获取DM电机反馈数据函数
 * @param[out]   dm_measure 达妙电机数据缓存
 * @param[in]    rx_data 指向包含反馈数据的数组指针
 * @note         从接收到的数据中提取DM电机的反馈信息，包括电机ID、状态、位置、速度、扭矩以及相关温度参数
 */
void DmFdbData(DmMeasure_s * dm_measure, uint8_t * rx_data)
{
    dm_measure->id = (rx_data[0]) & 0x0F;
    dm_measure->state = (rx_data[0]) >> 4;
    dm_measure->p_int = (rx_data[1] << 8) | rx_data[2];
    dm_measure->v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
    dm_measure->t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
    dm_measure->pos = uint_to_float(dm_measure->p_int, DM_P_MIN, DM_P_MAX, 16);  // (-12.5,12.5)
    dm_measure->vel = uint_to_float(dm_measure->v_int, DM_V_MIN, DM_V_MAX, 12);  // (-45.0,45.0)
    dm_measure->tor = uint_to_float(dm_measure->t_int, DM_T_MIN, DM_T_MAX, 12);  // (-18.0,18.0)
    dm_measure->t_mos = (float)(rx_data[6]);
    dm_measure->t_rotor = (float)(rx_data[7]);

    dm_measure->last_fdb_time = HAL_GetTick();
}

/**
 * @brief        DjiFdbData: 获取DJI电机反馈数据函数
 * @param[out]   dji_measure dji电机数据缓存
 * @param[in]    rx_data 反馈数据
 */
void DjiFdbData(DjiMotorMeasure_t * dji_measure, uint8_t * rx_data)
{
    dji_measure->last_ecd = dji_measure->ecd;
    dji_measure->ecd = (uint16_t)((rx_data)[0] << 8 | (rx_data)[1]);
    dji_measure->speed_rpm = (uint16_t)((rx_data)[2] << 8 | (rx_data)[3]);
    dji_measure->given_current = (uint16_t)((rx_data)[4] << 8 | (rx_data)[5]);
    dji_measure->temperate = (rx_data)[6];

    dji_measure->last_fdb_time = HAL_GetTick();
}

/**
 * @brief        LkFdbData: 获取LK电机反馈数据函数
 * @param[out]   dm_measure 电机数据缓存
 * @param[in]    rx_data 指向包含反馈数据的数组指针
 * @note         从接收到的数据中提取LK电机的反馈信息
 */
void LkFdbData(LkMeasure_s * lk_measure, uint8_t * rx_data)
{
    lk_measure->ctrl_id = rx_data[0];
    lk_measure->temprature = rx_data[1];
    lk_measure->iq = (uint16_t)(rx_data[3] << 8 | rx_data[2]);
    lk_measure->speed = (uint16_t)(rx_data[5] << 8 | rx_data[4]);
    lk_measure->encoder = (uint16_t)(rx_data[7] << 8 | rx_data[6]);

    lk_measure->last_fdb_time = HAL_GetTick();
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
    switch (rx_header->StdId) {  //电机解码
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
                DjiFdbData(&CAN1_DJI_MEASURE[i], rx_data);
            } else if (CAN == &hcan2)  // 接收到的数据是通过 CAN2 接收的
            {
                DjiFdbData(&CAN2_DJI_MEASURE[i], rx_data);
            }
            return;
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
            return;
        }
        case LK_M1_ID:
        case LK_M2_ID:
        case LK_M3_ID:
        case LK_M4_ID: {  // 以上ID为LK电机标识符
            static uint8_t i = 0;
            i = rx_header->StdId - LK_M1_ID;
            if (CAN == &hcan1)  // 接收到的数据是通过 CAN1 接收的
            {
                LkFdbData(&CAN1_LK_MEASURE[i], rx_data);
            } else if (CAN == &hcan2)  // 接收到的数据是通过 CAN2 接收的
            {
                LkFdbData(&CAN2_LK_MEASURE[i], rx_data);
            }
            return;
        }
        default: {
            break;
        }
    }

    //板间通信数据解码
    // clang-format off
    uint16_t data_type =  rx_header->StdId & 0xF00;
    uint16_t data_id   = (rx_header->StdId & 0x0F0) >> 4;
    uint16_t target_id =  rx_header->StdId & 0x00F;
    // clang-format on
    if (target_id != __SELF_BOARD_ID) return;

    if (data_type == BOARD_DATA_UINT16) {
        OTHER_BOARD_DATA_UINT16[data_id][0] = (rx_data[0] << 8) | rx_data[1];
        OTHER_BOARD_DATA_UINT16[data_id][1] = (rx_data[2] << 8) | rx_data[3];
        OTHER_BOARD_DATA_UINT16[data_id][2] = (rx_data[4] << 8) | rx_data[5];
        OTHER_BOARD_DATA_UINT16[data_id][3] = (rx_data[6] << 8) | rx_data[7];
    } else if (data_type == BOARD_DATA_ANY) {
        memcpy(OTHER_BOARD_DATA_ANY[data_id], rx_data, 8);
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
    p_motor->fdb.vel = ((float)decode_temp_mi - 32767.5f) / 32767.5f * 30.0f;

    decode_temp_mi = (rx_data[4] << 8 | rx_data[5]);
    p_motor->fdb.tor = ((float)decode_temp_mi - 32767.5f) / 32767.5f * 12.0f;

    decode_temp_mi = (rx_data[6] << 8 | rx_data[7]);
    p_motor->fdb.temp = (float)decode_temp_mi / 10.0f;
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

/**
 * @brief          获取DJI电机反馈数据
 * @param[out]     p_motor 电机结构体 
 * @param[in]      p_dji_motor_measure 电机反馈数据缓存区
 * @return         none
 */
static void GetDjiFdbData(Motor_s * p_motor, const DjiMotorMeasure_t * p_dji_motor_measure)
{
    p_motor->fdb.vel = p_dji_motor_measure->speed_rpm * RPM_TO_OMEGA * p_motor->reduction_ratio *
                       p_motor->direction;
    p_motor->fdb.pos = p_dji_motor_measure->ecd * 2 * M_PI / 8192 - M_PI;
    p_motor->fdb.temp = p_dji_motor_measure->temperate;
    p_motor->fdb.curr = p_dji_motor_measure->given_current;
    p_motor->fdb.ecd = p_dji_motor_measure->ecd;
}

/**
 * @brief          获取cybergear电机反馈数据
 * @param[out]     p_motor 电机结构体 
 * @param[in]      p_cybergear_measure 电机反馈数据缓存区
 * @return         none
 */
static void GetCybergearFdbData(Motor_s * p_motor, CybergearMeasure_s * p_cybergear_measure)
{
    CybergearRxDecode(p_motor, p_cybergear_measure->rx_data);
    RxCanInfoType_2_s * rx_info =
        (RxCanInfoType_2_s *)(&CAN1_CYBERGEAR_MEASURE[p_motor->id].ext_id);
    p_motor->fdb.state = rx_info->mode_state;
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
 * @brief          获取DM电机反馈数据
 * @param[out]     motor 电机结构体 
 * @param[in]      dm_measure 电机反馈数据缓存区
 * @return         none
 */
static void GetDmFdbData(Motor_s * motor, const DmMeasure_s * dm_measure)
{
    motor->fdb.pos = dm_measure->pos;
    motor->fdb.vel = dm_measure->vel;
    motor->fdb.tor = dm_measure->tor;
    motor->fdb.temp = dm_measure->t_mos;
    motor->fdb.state = dm_measure->state;

    uint32_t now = HAL_GetTick();
    if (now - dm_measure->last_fdb_time > MOTOR_STABLE_RUNNING_TIME) {
        motor->offline = true;
    } else {
        motor->offline = false;
    }
}

/**
 * @brief          获取LK电机反馈数据
 * @param[out]     motor 电机结构体 
 * @param[in]      lk_measure 电机反馈数据缓存区
 * @return         none
 */
static void GetLkFdbData(Motor_s * motor, const LkMeasure_s * lk_measure)
{
    motor->fdb.pos = uint_to_float(lk_measure->encoder, -M_PI, M_PI, 16);
    motor->fdb.vel = lk_measure->speed * DEGREE_TO_RAD;
    motor->fdb.curr = lk_measure->iq * MF_CONTROL_TO_CURRENT;
    motor->fdb.temp = lk_measure->temprature;

    uint32_t now = HAL_GetTick();
    if (now - lk_measure->last_fdb_time > MOTOR_STABLE_RUNNING_TIME) {
        motor->offline = true;
    } else {
        motor->offline = false;
    }
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
            GetDjiFdbData(p_motor, p_dji_motor_measure);
        } break;
        case DJI_M6020: {
            const DjiMotorMeasure_t * p_dji_motor_measure =
                GetDjiMotorMeasurePoint(p_motor->can, p_motor->id + 3);
            GetDjiFdbData(p_motor, p_dji_motor_measure);
        } break;
        case CYBERGEAR_MOTOR: {
            if (p_motor->can == 1) {
                GetCybergearFdbData(p_motor, &CAN1_CYBERGEAR_MEASURE[p_motor->id]);
            } else {
                GetCybergearFdbData(p_motor, &CAN2_CYBERGEAR_MEASURE[p_motor->id]);
            }
        } break;
        case DM_8009: {
            if (p_motor->can == 1) {
                GetDmFdbData(p_motor, &CAN1_DM_MEASURE[p_motor->id - 1]);
            } else {
                GetDmFdbData(p_motor, &CAN2_DM_MEASURE[p_motor->id - 1]);
            }
        } break;
        case MF_9025: {
            if (p_motor->can == 1) {
                GetLkFdbData(p_motor, &CAN1_LK_MEASURE[p_motor->id - 1]);
            } else {
                GetLkFdbData(p_motor, &CAN2_LK_MEASURE[p_motor->id - 1]);
            }
        } break;
        default:
            break;
    }
}

/**
 * @brief 获取板间通信数据
 * @param data_id 数据ID
 * @param data_offset 数据位置偏移
 * @return none
 */
uint16_t GetOtherBoardDataUint16(uint8_t data_id, uint8_t data_offset)
{
    return OTHER_BOARD_DATA_UINT16[data_id][data_offset];
}
