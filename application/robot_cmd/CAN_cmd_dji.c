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
 */
void CanCmdDjiMotor(
    uint8_t can, DJI_Std_ID std_id, int16_t curr_1, int16_t curr_2, int16_t curr_3, int16_t curr_4)
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

    CAN_SendTxMessage(CAN_CTRL_DATA.hcan, &CAN_CTRL_DATA.tx_header, CAN_CTRL_DATA.tx_data);
}

/*-------------------- 控制函数 --------------------*/

/**
 * @brief       dji电机速度控制
 * @param[in]   p_motor 
 * @param[in]   pid 
 * @param[in]   velocity 
 * @param[in]   feedforward 
 */
void DjiMotorVelocityControl(
    Motor_s * p_motor, pid_type_def * pid, float velocity, float feedforward)
{
    if (p_motor == NULL || pid == NULL) return;
    if (p_motor->type != DJI_M2006 && p_motor->type != DJI_M3508 && p_motor->type != DJI_M6020)
        return;
    p_motor->set.current = PID_calc(pid, p_motor->fdb.w, velocity) + feedforward;
}

/**
 * @brief       dji电机位置控制
 * @param[in]   p_motor 
 * @param[in]   pid 
 * @param[in]   velocity 
 * @param[in]   feedforward 
 */
void DjiMotorPositionControl(
    Motor_s * p_motor, pid_type_def * angle_pid, pid_type_def * velocity_pid, float angle,
    float feedforward)
{
    if (p_motor == NULL || angle_pid == NULL || velocity_pid == NULL) return;
    if (p_motor->type != DJI_M2006 && p_motor->type != DJI_M3508 && p_motor->type != DJI_M6020)
        return;
    float velocity_set = PID_calc(angle_pid, p_motor->fdb.pos, angle);
    float current_set = PID_calc(velocity_pid, p_motor->fdb.w, velocity_set) + feedforward;
    p_motor->set.current = current_set;
}
/************************ END OF FILE ************************/
