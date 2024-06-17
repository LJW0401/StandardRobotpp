/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       can_cmd_lingkong.c/h
  * @brief      CAN发送函数，通过CAN信号控制瓴控电机 MF9025.
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
#include "CAN_cmd_lingkong.h"

#include "bsp_can.h"
#include "stm32f4xx_hal.h"
#include "user_lib.h"

#define STDID_OFFESET ((uint16_t)0x140)

#define TORQUE_COEFFICIENT 0.32f                  // (N*m/A)转矩系数
#define CURRENT_TO_MULTICONTROL 62.5f             // (2000/32)(1/A)电流转换为控制量
#define CURRENT_TO_MF_CONTROL 124.1212121212121f  // (2048/16.5)(1/A)电流转换为控制量

static CanCtrlData_s CAN_CTRL_DATA = {
    .tx_header.IDE = CAN_ID_STD,
    .tx_header.RTR = CAN_RTR_DATA,
    .tx_header.DLC = 8,
};

/*-------------------- Private functions --------------------*/

/**
 * @brief        电机失能
 * @param[in]    hcan     指向CAN_HandleTypeDef结构的指针
 * @param[in]    motor_id 电机ID，指定目标电机
 */
static void DisableMotor(hcan_t * hcan, uint16_t motor_id)
{
    CAN_CTRL_DATA.hcan = hcan;

    CAN_CTRL_DATA.tx_header.StdId = motor_id + STDID_OFFESET;

    CAN_CTRL_DATA.tx_data[0] = 0x80;
    CAN_CTRL_DATA.tx_data[1] = 0x00;
    CAN_CTRL_DATA.tx_data[2] = 0x00;
    CAN_CTRL_DATA.tx_data[3] = 0x00;
    CAN_CTRL_DATA.tx_data[4] = 0x00;
    CAN_CTRL_DATA.tx_data[5] = 0x00;
    CAN_CTRL_DATA.tx_data[6] = 0x00;
    CAN_CTRL_DATA.tx_data[7] = 0x00;

    CAN_SendTxMessage(&CAN_CTRL_DATA);
}

/**
 * @brief        停止电机
 * @param[in]    hcan     指向CAN_HandleTypeDef结构的指针
 * @param[in]    motor_id 电机ID，指定目标电机
 */
static void StopMotor(hcan_t * hcan, uint16_t motor_id)
{
    CAN_CTRL_DATA.hcan = hcan;

    CAN_CTRL_DATA.tx_header.StdId = motor_id + STDID_OFFESET;

    CAN_CTRL_DATA.tx_data[0] = 0x81;
    CAN_CTRL_DATA.tx_data[1] = 0x00;
    CAN_CTRL_DATA.tx_data[2] = 0x00;
    CAN_CTRL_DATA.tx_data[3] = 0x00;
    CAN_CTRL_DATA.tx_data[4] = 0x00;
    CAN_CTRL_DATA.tx_data[5] = 0x00;
    CAN_CTRL_DATA.tx_data[6] = 0x00;
    CAN_CTRL_DATA.tx_data[7] = 0x00;

    CAN_SendTxMessage(&CAN_CTRL_DATA);
}

/**
 * @brief        电机使能
 * @param[in]    hcan     指向CAN_HandleTypeDef结构的指针
 * @param[in]    motor_id 电机ID，指定目标电机
 */
static void EnableMotor(hcan_t * hcan, uint16_t motor_id)
{
    CAN_CTRL_DATA.hcan = hcan;

    CAN_CTRL_DATA.tx_header.StdId = motor_id + STDID_OFFESET;

    CAN_CTRL_DATA.tx_data[0] = 0x88;
    CAN_CTRL_DATA.tx_data[1] = 0x00;
    CAN_CTRL_DATA.tx_data[2] = 0x00;
    CAN_CTRL_DATA.tx_data[3] = 0x00;
    CAN_CTRL_DATA.tx_data[4] = 0x00;
    CAN_CTRL_DATA.tx_data[5] = 0x00;
    CAN_CTRL_DATA.tx_data[6] = 0x00;
    CAN_CTRL_DATA.tx_data[7] = 0x00;

    CAN_SendTxMessage(&CAN_CTRL_DATA);
}

/**
 * @brief        单电机转矩闭环控制命令
 * @param[in]    hcan      指向CAN_HandleTypeDef结构的指针
 * @param[in]    motor_id  电机ID，指定目标电机
 * @param[in]    iqControl 转矩电流 -2048~2048
 */
static void SingleTorqueControl(hcan_t * hcan, uint16_t motor_id, int16_t iqControl)
{
    CAN_CTRL_DATA.hcan = hcan;

    CAN_CTRL_DATA.tx_header.StdId = motor_id + STDID_OFFESET;

    CAN_CTRL_DATA.tx_data[0] = 0xA1;
    CAN_CTRL_DATA.tx_data[1] = 0x00;
    CAN_CTRL_DATA.tx_data[2] = 0x00;
    CAN_CTRL_DATA.tx_data[3] = 0x00;
    CAN_CTRL_DATA.tx_data[4] = *(uint8_t *)(&iqControl);
    CAN_CTRL_DATA.tx_data[5] = *((uint8_t *)(&iqControl) + 1);
    CAN_CTRL_DATA.tx_data[6] = 0x00;
    CAN_CTRL_DATA.tx_data[7] = 0x00;

    CAN_SendTxMessage(&CAN_CTRL_DATA);
}

/**
 * @brief        单电机速度闭环控制命令
 * @param[in]    hcan      指向CAN_HandleTypeDef结构的指针
 * @param[in]    motor_id  电机ID，指定目标电机
 * @param[in]    speedControl 
 */
static void SingleSpeedControl(hcan_t * hcan, uint16_t motor_id, int32_t speedControl)
{
    CAN_CTRL_DATA.hcan = hcan;

    CAN_CTRL_DATA.tx_header.StdId = motor_id + STDID_OFFESET;

    CAN_CTRL_DATA.tx_data[0] = 0xA2;
    CAN_CTRL_DATA.tx_data[1] = 0x00;
    CAN_CTRL_DATA.tx_data[2] = 0x00;
    CAN_CTRL_DATA.tx_data[3] = 0x00;
    CAN_CTRL_DATA.tx_data[4] = *(uint8_t *)(&speedControl);
    CAN_CTRL_DATA.tx_data[5] = *((uint8_t *)(&speedControl) + 1);
    CAN_CTRL_DATA.tx_data[6] = *((uint8_t *)(&speedControl) + 2);
    CAN_CTRL_DATA.tx_data[7] = *((uint8_t *)(&speedControl) + 3);

    CAN_SendTxMessage(&CAN_CTRL_DATA);
}

/**
 * @brief        多电机转矩闭环控制命令
 * @param[in]    hcan        指向CAN_HandleTypeDef结构的指针
 * @param[in]    iqControl_1 转矩电流 -2000\\~2000
 * @param[in]    iqControl_2 转矩电流 -2000\\~2000
 * @param[in]    iqControl_3 转矩电流 -2000\\~2000
 * @param[in]    iqControl_4 转矩电流 -2000\\~2000
 */
static void MultipleTorqueControl(
    hcan_t * hcan, int16_t iqControl_1, int16_t iqControl_2, int16_t iqControl_3,
    int16_t iqControl_4)
{
    CAN_CTRL_DATA.hcan = hcan;

    CAN_CTRL_DATA.tx_header.StdId = 0x280;

    CAN_CTRL_DATA.tx_data[0] = *(uint8_t *)(&iqControl_1);
    CAN_CTRL_DATA.tx_data[1] = *((uint8_t *)(&iqControl_1) + 1);
    CAN_CTRL_DATA.tx_data[2] = *(uint8_t *)(&iqControl_2);
    CAN_CTRL_DATA.tx_data[3] = *((uint8_t *)(&iqControl_2) + 1);
    CAN_CTRL_DATA.tx_data[4] = *(uint8_t *)(&iqControl_3);
    CAN_CTRL_DATA.tx_data[5] = *((uint8_t *)(&iqControl_3) + 1);
    CAN_CTRL_DATA.tx_data[6] = *(uint8_t *)(&iqControl_4);
    CAN_CTRL_DATA.tx_data[7] = *((uint8_t *)(&iqControl_4) + 1);

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
    if (motor->type != MF_9025) return NULL;

    if (motor->can == 1)
        return &hcan1;
    else if (motor->can == 2)
        return &hcan2;

    return NULL;
}

/*-------------------- User functions --------------------*/

void LkDisable(Motor_s * p_motor)
{
    hcan_t * hcan = GetHcanPoint(p_motor);
    if (hcan == NULL) return;

    DisableMotor(hcan, p_motor->id);
}

void LkStop(Motor_s * p_motor)
{
    hcan_t * hcan = GetHcanPoint(p_motor);
    if (hcan == NULL) return;

    StopMotor(hcan, p_motor->id);
}

void LkEnable(Motor_s * p_motor)
{
    hcan_t * hcan = GetHcanPoint(p_motor);
    if (hcan == NULL) return;

    EnableMotor(hcan, p_motor->id);
}

void LkSingleTorqueControl(Motor_s * p_motor)
{
    hcan_t * hcan = GetHcanPoint(p_motor);
    if (hcan == NULL) return;

    SingleTorqueControl(
        hcan, p_motor->id,
        fp32_constrain(p_motor->set.tor, LK_MIN_MF_TORQUE, LK_MAX_MF_TORQUE) / TORQUE_COEFFICIENT *
            CURRENT_TO_MF_CONTROL);
}

void LkSingleSpeedControl(Motor_s * p_motor)
{
    hcan_t * hcan = GetHcanPoint(p_motor);
    if (hcan == NULL) return;

    SingleSpeedControl(hcan, p_motor->id, p_motor->set.vel * RAD_TO_DEGREE * 100);
}

void LkMultipleTorqueControl(
    uint8_t can, float torque_1, float torque_2, float torque_3, float torque_4)
{
    hcan_t * hcan = NULL;
    if (can == 1)
        hcan = &hcan1;
    else if (can == 2)
        hcan = &hcan2;

    if (hcan == NULL) return;

    int16_t current[4];
    current[0] = fp32_constrain(torque_1, LK_MIN_MF_TORQUE, LK_MAX_MF_TORQUE) / TORQUE_COEFFICIENT;
    current[1] = fp32_constrain(torque_2, LK_MIN_MF_TORQUE, LK_MAX_MF_TORQUE) / TORQUE_COEFFICIENT;
    current[2] = fp32_constrain(torque_3, LK_MIN_MF_TORQUE, LK_MAX_MF_TORQUE) / TORQUE_COEFFICIENT;
    current[3] = fp32_constrain(torque_4, LK_MIN_MF_TORQUE, LK_MAX_MF_TORQUE) / TORQUE_COEFFICIENT;

    MultipleTorqueControl(
        hcan, current[0] * CURRENT_TO_MULTICONTROL, current[1] * CURRENT_TO_MULTICONTROL,
        current[2] * CURRENT_TO_MULTICONTROL, current[3] * CURRENT_TO_MULTICONTROL);
}

void LkMultipleIqControl(
    uint8_t can, int16_t iqControl_1, int16_t iqControl_2, int16_t iqControl_3, int16_t iqControl_4)
{
    hcan_t * hcan = NULL;
    if (can == 1)
        hcan = &hcan1;
    else if (can == 2)
        hcan = &hcan2;

    if (hcan == NULL) return;

    int16_t current[4];
    current[0] = int16_constrain(iqControl_1, -2000, 2000);
    current[1] = int16_constrain(iqControl_2, -2000, 2000);
    current[2] = int16_constrain(iqControl_3, -2000, 2000);
    current[3] = int16_constrain(iqControl_4, -2000, 2000);

    MultipleTorqueControl(hcan, current[0], current[1], current[2], current[3]);
}
/************************ END OF FILE ************************/
