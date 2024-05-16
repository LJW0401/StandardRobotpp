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

/*-------------------- Function --------------------*/

/**
 * @brief          获取发送数据缓冲区指针
 * @param[in]      can 发送数据使用的can口(1/2)
 * @param[in]      std_id 发送数据使用的std_id
 * @return         发送数据缓冲区指针
 */
static DJI_Motor_Send_Data_s * GetSendDataBufferPoint(uint8_t can, DJI_Std_ID std_id)
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
    return dji_motor_send_data;
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
    DJI_Motor_Send_Data_s * dji_motor_send_data = GetSendDataBufferPoint(can, std_id);
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

/**
 * @brief 添加信息到发送数据缓冲区
 * @param p_motor 电机结构体
 * @param std_id 电机控制数据标准ID
 */
void AddDjiMotorSendData(Motor_s * p_motor, DJI_Std_ID std_id)
{
    if (p_motor->type == DJI_M2006 || p_motor->type == DJI_M3508 || p_motor->type == DJI_M6020)
        return;

    DJI_Motor_Send_Data_s * dji_motor_send_data = GetSendDataBufferPoint(p_motor->can, std_id);
    if (dji_motor_send_data == NULL) return;

    dji_motor_send_data->tx_message.StdId = dji_motor_send_data->std_id;
    dji_motor_send_data->tx_message.IDE = CAN_ID_STD;
    dji_motor_send_data->tx_message.RTR = CAN_RTR_DATA;
    dji_motor_send_data->tx_message.DLC = 0x08;

    uint8_t offset = ((p_motor->id - 1) % 4) * 2;
    int16_t current_set = p_motor->set.current;

    dji_motor_send_data->can_send_data[offset] = (current_set >> 8);
    dji_motor_send_data->can_send_data[offset + 1] = current_set;
}

/**
 * @brief 发送电机控制数据
 * @param can 1/2
 * @param std_id 电机控制数据标准ID
 */
void SendDjiMotorCmdData(uint8_t can, DJI_Std_ID std_id)
{
    DJI_Motor_Send_Data_s * dji_motor_send_data = GetSendDataBufferPoint(can, std_id);
    if (dji_motor_send_data == NULL) return;
    CAN_SendTxMessage(
        dji_motor_send_data->CAN, &dji_motor_send_data->tx_message,
        dji_motor_send_data->can_send_data);
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
