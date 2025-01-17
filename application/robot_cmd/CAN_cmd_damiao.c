/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       can_cmd_dm.c/h
  * @brief      CAN发送函数，通过CAN信号控制达妙电机 8009.
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
#include "CAN_cmd_damiao.h"

#include "bsp_can.h"
#include "motor.h"
#include "stdbool.h"
#include "stm32f4xx_hal.h"
#include "struct_typedef.h"
#include "user_lib.h"

// 电机参数设置结构体

static CanCtrlData_s CAN_CTRL_DATA = {
    .tx_header.IDE = CAN_ID_STD,
    .tx_header.RTR = CAN_RTR_DATA,
    .tx_header.DLC = 8,
};

/*-------------------- Private functions --------------------*/

/**
************************************************************************
* @brief      	ClearErr: 清除电机错误函数
* @param[in]    hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]    motor_id: 电机ID，指定目标电机
* @param[in]    mode_id:  模式ID，指定要清除错误的模式
* @retval     	void
* @details    	通过CAN总线向特定电机发送清除错误的命令。
************************************************************************
**/
static void ClearErr(hcan_t * hcan, uint16_t motor_id, uint16_t mode_id)
{
    CAN_CTRL_DATA.hcan = hcan;

    CAN_CTRL_DATA.tx_header.StdId = motor_id + mode_id;

    CAN_CTRL_DATA.tx_data[0] = 0xFF;
    CAN_CTRL_DATA.tx_data[1] = 0xFF;
    CAN_CTRL_DATA.tx_data[2] = 0xFF;
    CAN_CTRL_DATA.tx_data[3] = 0xFF;
    CAN_CTRL_DATA.tx_data[4] = 0xFF;
    CAN_CTRL_DATA.tx_data[5] = 0xFF;
    CAN_CTRL_DATA.tx_data[6] = 0xFF;
    CAN_CTRL_DATA.tx_data[7] = 0xFB;

    CAN_SendTxMessage(&CAN_CTRL_DATA);
}

/**
************************************************************************
* @brief      	EnableMotorMode: 启用电机模式函数
* @param[in]    hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]    motor_id: 电机ID，指定目标电机
* @param[in]    mode_id:  模式ID，指定要开启的模式
* @retval     	void
* @details    	通过CAN总线向特定电机发送启用特定模式的命令
************************************************************************
**/
static void EnableMotorMode(hcan_t * hcan, uint16_t motor_id, uint16_t mode_id)
{
    CAN_CTRL_DATA.hcan = hcan;

    CAN_CTRL_DATA.tx_header.StdId = motor_id + mode_id;

    CAN_CTRL_DATA.tx_data[0] = 0xFF;
    CAN_CTRL_DATA.tx_data[1] = 0xFF;
    CAN_CTRL_DATA.tx_data[2] = 0xFF;
    CAN_CTRL_DATA.tx_data[3] = 0xFF;
    CAN_CTRL_DATA.tx_data[4] = 0xFF;
    CAN_CTRL_DATA.tx_data[5] = 0xFF;
    CAN_CTRL_DATA.tx_data[6] = 0xFF;
    CAN_CTRL_DATA.tx_data[7] = 0xFC;

    CAN_SendTxMessage(&CAN_CTRL_DATA);
}

/**
************************************************************************
* @brief      	DisableMotorMode: 禁用电机模式函数
* @param[in]    hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]    motor_id: 电机ID，指定目标电机
* @param[in]    mode_id:  模式ID，指定要禁用的模式
* @retval     	void
* @details    	通过CAN总线向特定电机发送禁用特定模式的命令
************************************************************************
**/
static void DisableMotorMode(hcan_t * hcan, uint16_t motor_id, uint16_t mode_id)
{
    CAN_CTRL_DATA.hcan = hcan;

    CAN_CTRL_DATA.tx_header.StdId = motor_id + mode_id;

    CAN_CTRL_DATA.tx_data[0] = 0xFF;
    CAN_CTRL_DATA.tx_data[1] = 0xFF;
    CAN_CTRL_DATA.tx_data[2] = 0xFF;
    CAN_CTRL_DATA.tx_data[3] = 0xFF;
    CAN_CTRL_DATA.tx_data[4] = 0xFF;
    CAN_CTRL_DATA.tx_data[5] = 0xFF;
    CAN_CTRL_DATA.tx_data[6] = 0xFF;
    CAN_CTRL_DATA.tx_data[7] = 0xFD;

    CAN_SendTxMessage(&CAN_CTRL_DATA);
}

/**
************************************************************************
* @brief      	SavePosZero: 保存位置零点函数
* @param[in]    hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]    motor_id: 电机ID，指定目标电机
* @param[in]    mode_id:  模式ID，指定要保存位置零点的模式
* @retval     	void
* @details    	通过CAN总线向特定电机发送保存位置零点的命令
************************************************************************
**/
void SavePosZero(hcan_t * hcan, uint16_t motor_id, uint16_t mode_id)
{
    CAN_CTRL_DATA.hcan = hcan;

    CAN_CTRL_DATA.tx_header.StdId = motor_id + mode_id;

    CAN_CTRL_DATA.tx_data[0] = 0xFF;
    CAN_CTRL_DATA.tx_data[1] = 0xFF;
    CAN_CTRL_DATA.tx_data[2] = 0xFF;
    CAN_CTRL_DATA.tx_data[3] = 0xFF;
    CAN_CTRL_DATA.tx_data[4] = 0xFF;
    CAN_CTRL_DATA.tx_data[5] = 0xFF;
    CAN_CTRL_DATA.tx_data[6] = 0xFF;
    CAN_CTRL_DATA.tx_data[7] = 0xFE;

    CAN_SendTxMessage(&CAN_CTRL_DATA);
}

/**
************************************************************************
* @brief      	MitCtrl: MIT模式下的电机控制函数
* @param[in]    hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]    motor_id:	    电机ID，指定目标电机
* @param[in]    pos:			位置给定值
* @param[in]    vel:			速度给定值
* @param[in]    kp:				位置比例系数
* @param[in]    kd:				位置微分系数
* @param[in]    torq:			转矩给定值
* @retval     	void
* @details    	通过CAN总线向电机发送MIT模式下的控制帧。
************************************************************************
**/
static void MitCtrl(
    hcan_t * hcan, uint16_t motor_id, float pos, float vel, float kp, float kd, float torq)
{
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;

    CAN_CTRL_DATA.hcan = hcan;
    
    CAN_CTRL_DATA.tx_header.StdId = motor_id + DM_MODE_MIT;

    pos_tmp = float_to_uint(pos, DM_P_MIN, DM_P_MAX, 16);
    vel_tmp = float_to_uint(vel, DM_V_MIN, DM_V_MAX, 12);
    kp_tmp = float_to_uint(kp, DM_KP_MIN, DM_KP_MAX, 12);
    kd_tmp = float_to_uint(kd, DM_KD_MIN, DM_KD_MAX, 12);
    tor_tmp = float_to_uint(torq, DM_T_MIN, DM_T_MAX, 12);

    CAN_CTRL_DATA.tx_data[0] = (pos_tmp >> 8);
    CAN_CTRL_DATA.tx_data[1] = pos_tmp;
    CAN_CTRL_DATA.tx_data[2] = (vel_tmp >> 4);
    CAN_CTRL_DATA.tx_data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    CAN_CTRL_DATA.tx_data[4] = kp_tmp;
    CAN_CTRL_DATA.tx_data[5] = (kd_tmp >> 4);
    CAN_CTRL_DATA.tx_data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    CAN_CTRL_DATA.tx_data[7] = tor_tmp;

    CAN_SendTxMessage(&CAN_CTRL_DATA);
}

/**
************************************************************************
* @brief      	PosSpeedCtrl: 位置速度控制函数
* @param[in]   hcan:		指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]   motor_id:	电机ID，指定目标电机
* @param[in]   vel:			速度给定值
* @retval     	void
* @details    	通过CAN总线向电机发送位置速度控制命令
************************************************************************
**/
static void PosSpeedCtrl(hcan_t * hcan, uint16_t motor_id, float pos, float vel)
{
    CAN_CTRL_DATA.hcan = hcan;

    CAN_CTRL_DATA.tx_header.StdId = motor_id + DM_MODE_POS;
    CAN_CTRL_DATA.tx_header.DLC = 8;

    uint8_t *pbuf, *vbuf;
    pbuf = (uint8_t *)&pos;
    vbuf = (uint8_t *)&vel;

    CAN_CTRL_DATA.tx_data[0] = *pbuf;
    CAN_CTRL_DATA.tx_data[1] = *(pbuf + 1);
    CAN_CTRL_DATA.tx_data[2] = *(pbuf + 2);
    CAN_CTRL_DATA.tx_data[3] = *(pbuf + 3);

    CAN_CTRL_DATA.tx_data[4] = *vbuf;
    CAN_CTRL_DATA.tx_data[5] = *(vbuf + 1);
    CAN_CTRL_DATA.tx_data[6] = *(vbuf + 2);
    CAN_CTRL_DATA.tx_data[7] = *(vbuf + 3);

    CAN_SendTxMessage(&CAN_CTRL_DATA);
}

/**
************************************************************************
* @brief      	SpeedCtrl: 速度控制函数
* @param[in]    hcan: 	  指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]    motor_id: 电机ID，指定目标电机
* @param[in]    vel: 	  速度给定值
* @retval     	void
* @details    	通过CAN总线向电机发送速度控制命令
************************************************************************
**/
static void SpeedCtrl(hcan_t * hcan, uint16_t motor_id, float vel)
{
    CAN_CTRL_DATA.hcan = hcan;

    CAN_CTRL_DATA.tx_header.StdId = motor_id + DM_MODE_SPEED;
    CAN_CTRL_DATA.tx_header.DLC = 4;

    uint8_t * vbuf;
    vbuf = (uint8_t *)&vel;

    CAN_CTRL_DATA.tx_data[0] = *vbuf;
    CAN_CTRL_DATA.tx_data[1] = *(vbuf + 1);
    CAN_CTRL_DATA.tx_data[2] = *(vbuf + 2);
    CAN_CTRL_DATA.tx_data[3] = *(vbuf + 3);

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
    if (motor->type != DM_8009) return NULL;

    if (motor->can == 1)
        return &hcan1;
    else if (motor->can == 2)
        return &hcan2;

    return NULL;
}

/*-------------------- User functions --------------------*/
/**
 * @brief          达妙电机清除错误
 * @param[in]      motor 电机结构体
 * @param[in]      mode_id 模式ID
 * @retval         none
 */
void DmClearErr(Motor_s * motor)
{
    hcan_t * hcan = GetHcanPoint(motor);
    if (hcan == NULL) return;

    ClearErr(hcan, motor->id, motor->mode);
}

/**
 * @brief          达妙电机使能
 * @param[in]      motor 电机结构体
 * @param[in]      mode_id 模式ID
 * @retval         none
 */
void DmEnable(Motor_s * motor)
{
    hcan_t * hcan = GetHcanPoint(motor);
    if (hcan == NULL) return;

    EnableMotorMode(hcan, motor->id, motor->mode);
}

/**
 * @brief          达妙电机失能
 * @param[in]      motor 电机结构体
 * @param[in]      mode_id 模式ID
 * @retval         none
 */
void DmDisable(Motor_s * motor)
{
    hcan_t * hcan = GetHcanPoint(motor);
    if (hcan == NULL) return;

    DisableMotorMode(hcan, motor->id, motor->mode);
}

/**
 * @brief          达妙电机保存零点
 * @param[in]      motor 电机结构体
 * @param[in]      mode_id 模式ID
 * @retval         none
 */
void DmSavePosZero(Motor_s * motor)
{
    hcan_t * hcan = GetHcanPoint(motor);
    if (hcan == NULL) return;

    SavePosZero(hcan, motor->id, motor->mode);
}

/**
 * @brief          达妙电机停止，直接发送0力矩
 * @param[in]      motor 电机结构体
 * @retval         none
 */
void DmMitStop(Motor_s * motor)
{
    hcan_t * hcan = GetHcanPoint(motor);
    if (hcan == NULL) return;

    MitCtrl(hcan, motor->id, 0, 0, 0, 0, 0);
}

/**
 * @brief          达妙电机使用力矩控制
 * @param[in]      motor 电机结构体
 * @retval         none
 */
void DmMitCtrlTorque(Motor_s * motor)
{
    hcan_t * hcan = GetHcanPoint(motor);
    if (hcan == NULL) return;

    MitCtrl(hcan, motor->id, 0, 0, 0, 0, motor->set.tor);
}

/**
 * @brief          达妙电机使用MIT模式控制速度
 * @param[in]      motor 电机结构体
 * @retval         none
 */
void DmMitCtrlVelocity(Motor_s * motor, float kd)
{
    hcan_t * hcan = GetHcanPoint(motor);
    if (hcan == NULL) return;

    MitCtrl(hcan, motor->id, 0, motor->set.vel, 0, kd, 0);
}

/**
 * @brief          达妙电机使用MIT模式控制位置
 * @param[in]      motor 电机结构体
 * @retval         none
 */
void DmMitCtrlPosition(Motor_s * motor, float kp, float kd)
{
    hcan_t * hcan = GetHcanPoint(motor);
    if (hcan == NULL) return;

    MitCtrl(hcan, motor->id, motor->set.pos, 0, kp, kd, 0);
}

/**
 * @brief          达妙电机位置模式控制
 * @param[in]      motor 电机结构体
 * @retval         none
 */
void DmPosCtrl(Motor_s * motor)
{
    hcan_t * hcan = GetHcanPoint(motor);
    if (hcan == NULL) return;

    PosSpeedCtrl(hcan, motor->id, motor->set.pos, motor->set.vel);
}

/**
 * @brief          达妙电机速度模式控制
 * @param[in]      motor 电机结构体
 * @retval         none
 */
void DmSpeedCtrl(Motor_s * motor)
{
    hcan_t * hcan = GetHcanPoint(motor);
    if (hcan == NULL) return;

    SpeedCtrl(hcan, motor->id, motor->set.vel);
}
/************************ END OF FILE ************************/
