/**
  ****************************(C) COPYRIGHT 2023 Polarbear*************************
  * @file       usb_task.c/h
  * @brief      usb outputs the IMU and gimbal data to the miniPC
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2023-7-11       Penguin         1. done
  *  V1.0.1     Oct-31-2023     LihanChen       1. 完成核心框架的构建，使其与调试模式和 miniPC 模式兼容。
  *  V1.0.2     Nov-01-2023     LihanChen       1. 将 Append_CRC16_Check_Sum_SendPacketVision() 和 Append_CRC-16_Check_Stum_OutputPCData() 合并到 Append_CRC16_Check_Sum() 中
  *  v2.0.0     Feb-24-2024     LihanChen       1. 重构代码，将 usb_task 的发送和接收分离，并实现分包发送和接收视觉和导航数据
  *  v2.0.1     Mar-02-2024     LihanChen       1. 删除未使用的 USB_RECEIVE_STATE 定义，去除编译警告
  *  v2.1.0     Mar-04-2024     LihanChen       1. 添加裁判系统信息转发（保留接口，待填入真实数值）
  *  v2.1.1     Mar-24-2024     LihanChen       1. 简化Append_CRC16_Check_Sum()函数，移除usb_printf()函数
  *  v2.1.2     Mar-25-2024     Penguin         1. 优化了数据接收逻辑，添加了数据指针获取函数
  *                                             2. 通过CRC8_CRC16.c/h文件中的函数实现CRC8校验和CRC16校验
  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 Polarbear*************************
*/

#include "usb_task.h"

#include <stdio.h>
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_conf.h"
#include "cmsis_os.h"
#include "bsp_buzzer.h"
#include "INS_task.h"
#include "referee.h"
#include "CRC8_CRC16.h"

// Constants
#define OUTPUT_VISION_STATE 0
#define OUTPUT_PC_STATE 1

#define EXPECTED_INPUT_NAVIGATION_HEDER 0xA4
#define EXPECTED_INPUT_VISION_HEDER 0xA5
#define EXPECTED_INPUT_PC_HEDER 0xA6
#define SET_OUTPUT_VISION_HEDER 0x5A
#define SET_OUTPUT_AllRobotHP_HEDER 0x5B
#define SET_OUTPUT_GameStatus_HEDER 0x5C
#define SET_OUTPUT_RobotStatus_HEDER 0x5D

#define CRC16_INIT 0xFFFF
#define USB_STATE_INVALID 0xFF
#define APP_RX_DATA_SIZE 2048
#define APP_TX_DATA_SIZE 2048
#define USB_RECEIVE_LEN 384

// Variable Declarations
static uint8_t usb_tx_buf[APP_TX_DATA_SIZE];
static uint8_t usb_rx_buf[APP_RX_DATA_SIZE];

static ReceivedPacketVision_s ReceivedPacketVision;
static ReceivedPacketTwist_s ReceivedPacketTwist;
static InputPCData_s InputPCData;

static SendPacketVision_s SendPacketVision;
static SendPacketAllRobotHP_s SendPacketAllRobotHP;
static SendPacketGameStatus_s SendPacketGameStatus;
static SendPacketRobotStatus_s SendPacketRobotStatus;
OutputPCData_s OutputPCData;

static uint8_t USB_SEND_STATE;
static const fp32 *gimbal_INT_gyro_angle_point;

// Function Prototypes
static void usb_send_vision(void);
static void usb_send_AllRobotHP(void);
static void usb_send_GameStatus(void);
static void usb_send_RobotStatus(void);
static void usb_send_outputPC(void);
static void usb_receive_navigation(void);
static void usb_receive_vision(void);
static void usb_receive_PC(void);
static void usb_receive(void);
static void char_to_uint(uint8_t *word, const char *str);

/**
 * @brief      USB任务主函数，switch中为发送，接收部分在usb_receive()中处理
 * @param[in]  argument: 任务参数
 * @retval     None
 */
void usb_task(void const *argument)
{
    MX_USB_DEVICE_Init();
    while (1)
    {
        usb_receive();

        switch (USB_SEND_STATE)
        {
        case OUTPUT_VISION_STATE:
            usb_send_vision();

            usb_send_AllRobotHP();
            usb_send_GameStatus();
            usb_send_RobotStatus();
            break;

        case OUTPUT_PC_STATE:
            usb_send_outputPC();
            break;

        case USB_STATE_INVALID:
            break;
        }
        osDelay(7); // 为了减少上位机处理数据的负担，降低发送频率。 Delay for 6.67 milliseconds (150Hz)
    }
}

/**
 * @brief      USB接收主函数，处理接收到的原始数据，根据数据头判断数据类型并分包处理
 * @param      None
 * @retval     None
 */
static void usb_receive(void)
{
    uint32_t len = USB_RECEIVE_LEN;
    CDC_Receive_FS(usb_rx_buf, &len); // Read data into the buffer

    switch (usb_rx_buf[0])
    {
    case EXPECTED_INPUT_NAVIGATION_HEDER:
        /* 导航 */
        usb_receive_navigation();
        break;

    case EXPECTED_INPUT_VISION_HEDER:
        /* 自瞄 */
        USB_SEND_STATE = OUTPUT_VISION_STATE;
        usb_receive_vision();
        break;

    case EXPECTED_INPUT_PC_HEDER:
        /* LJW串口助手 https://gitee.com/SMBU-POLARBEAR/Serial_Port_Assistant */
        USB_SEND_STATE = OUTPUT_PC_STATE;
        usb_receive_PC();
        break;

    default:
        break;
    }
}

/**
 * @brief      为视觉部分发送数据
 * @param      None
 * @retval     None
 */
static void usb_send_vision(void)
{
    SendPacketVision.header = SET_OUTPUT_VISION_HEDER; // 别放入 crc_ok

    gimbal_INT_gyro_angle_point = get_INS_angle_point(); // 获取云台IMU欧拉角：0:yaw, 1:pitch, 2:roll（弧度）

    SendPacketVision.detect_color = !get_team_color(); // TODO: 由裁判系统赋值，0-Red, 1-Blue
    SendPacketVision.reset_tracker = 0;                // TODO: 由自瞄模式赋值，是否重置追踪器
    // SendPacketVision.roll = gimbal_INT_gyro_angle_point[2];
    // SendPacketVision.pitch = gimbal_INT_gyro_angle_point[1];
    // SendPacketVision.yaw = gimbal_INT_gyro_angle_point[0];
    // SendPacketVision.aim_x = 0; // TODO: SendPacketVision.aim_x 由自瞄模式赋值，主要用于上位机可视化与自瞄调试，不影响实际功能
    // SendPacketVision.aim_y = 0; // TODO: SendPacketVision.aim_y 同上
    // SendPacketVision.aim_z = 0; // TODO: SendPacketVision.aim_y 同上

    append_CRC16_check_sum((uint8_t *)&SendPacketVision, sizeof(SendPacketVision));
    memcpy(usb_tx_buf, &SendPacketVision, sizeof(SendPacketVision_s));
    CDC_Transmit_FS(usb_tx_buf, sizeof(SendPacketVision_s));
}

/**
 * @brief      转发裁判系统，为上位机发送敌我双方全体机器人血量信息
 * @param      None
 * @retval     None
 */
static void usb_send_AllRobotHP(void)
{
    SendPacketAllRobotHP.header = SET_OUTPUT_AllRobotHP_HEDER;

    SendPacketAllRobotHP.red_1_robot_hp = game_robot_HP.red_1_robot_HP;
    SendPacketAllRobotHP.red_2_robot_hp = game_robot_HP.red_2_robot_HP;
    SendPacketAllRobotHP.red_3_robot_hp = game_robot_HP.red_3_robot_HP;
    SendPacketAllRobotHP.red_4_robot_hp = game_robot_HP.red_4_robot_HP;
    SendPacketAllRobotHP.red_5_robot_hp = game_robot_HP.red_5_robot_HP;
    SendPacketAllRobotHP.red_7_robot_hp = game_robot_HP.red_7_robot_HP;
    SendPacketAllRobotHP.red_outpost_hp = game_robot_HP.red_outpost_HP;
    SendPacketAllRobotHP.red_base_hp = game_robot_HP.red_base_HP;
    SendPacketAllRobotHP.blue_1_robot_hp = game_robot_HP.blue_1_robot_HP;
    SendPacketAllRobotHP.blue_2_robot_hp = game_robot_HP.blue_2_robot_HP;
    SendPacketAllRobotHP.blue_3_robot_hp = game_robot_HP.blue_3_robot_HP;
    SendPacketAllRobotHP.blue_4_robot_hp = game_robot_HP.blue_4_robot_HP;
    SendPacketAllRobotHP.blue_5_robot_hp = game_robot_HP.blue_5_robot_HP;
    SendPacketAllRobotHP.blue_7_robot_hp = game_robot_HP.blue_7_robot_HP;
    SendPacketAllRobotHP.blue_outpost_hp = game_robot_HP.blue_outpost_HP;
    SendPacketAllRobotHP.blue_base_hp = game_robot_HP.blue_base_HP;

    append_CRC16_check_sum((uint8_t *)&SendPacketAllRobotHP, sizeof(SendPacketAllRobotHP_s));
    memcpy(usb_tx_buf, &SendPacketAllRobotHP, sizeof(SendPacketAllRobotHP_s));
    CDC_Transmit_FS(usb_tx_buf, sizeof(SendPacketAllRobotHP_s));
}

/**
 * @brief      转发裁判系统，为上位机发送比赛状态信息
 * @param      None
 * @retval     None
 */
static void usb_send_GameStatus(void)
{
    SendPacketGameStatus.header = SET_OUTPUT_GameStatus_HEDER;

    SendPacketGameStatus.game_progress = game_status.game_progress;
    SendPacketGameStatus.stage_remain_time = game_status.stage_remain_time;

    append_CRC16_check_sum((uint8_t *)&SendPacketGameStatus, sizeof(SendPacketGameStatus_s));
    memcpy(usb_tx_buf, &SendPacketGameStatus, sizeof(SendPacketGameStatus_s));
    CDC_Transmit_FS(usb_tx_buf, sizeof(SendPacketGameStatus_s));
}

/**
 * @brief      转发裁判系统，为上位机发送机器人状态信息
 * @param      None
 * @retval     None
 */
static void usb_send_RobotStatus(void)
{
    SendPacketRobotStatus.header = SET_OUTPUT_RobotStatus_HEDER;

    SendPacketRobotStatus.robot_id = robot_status.robot_id;
    SendPacketRobotStatus.current_hp = robot_status.current_HP;
    SendPacketRobotStatus.shooter_heat = get_shoot_heat();
    SendPacketRobotStatus.team_color = get_team_color();
    SendPacketRobotStatus.is_attacked = 0; // TODO: 由裁判系统赋值

    append_CRC16_check_sum((uint8_t *)&SendPacketRobotStatus, sizeof(SendPacketRobotStatus_s));
    memcpy(usb_tx_buf, &SendPacketRobotStatus, sizeof(SendPacketRobotStatus_s));
    CDC_Transmit_FS(usb_tx_buf, sizeof(SendPacketRobotStatus_s));
}

/**
 * @brief      发送数据到LJW的串口调试助手
 * @param      None
 * @retval     None
 */
static void usb_send_outputPC(void)
{
    gimbal_INT_gyro_angle_point = get_INS_angle_point();
    const fp32 * accel_point = get_accel_data_point();
    const Angle_t *angle = GetAnglePoint();
    const Accel_t *accel = GetAccelPoint();

    OutputPCData.header = 0x6A;
    OutputPCData.length = sizeof(OutputPCData_s);

    char_to_uint(OutputPCData.name_1, "o_roll");
    OutputPCData.type_1 = 1;
    OutputPCData.data_1 = gimbal_INT_gyro_angle_point[2];

    char_to_uint(OutputPCData.name_2, "o_pitch");
    OutputPCData.type_2 = 1;
    OutputPCData.data_2 = gimbal_INT_gyro_angle_point[1];

    char_to_uint(OutputPCData.name_3, "o_yaw");
    OutputPCData.type_3 = 1;
    OutputPCData.data_3 = gimbal_INT_gyro_angle_point[0];

    char_to_uint(OutputPCData.name_4, "gx");
    OutputPCData.type_4 = 1;
    OutputPCData.data_4 = accel->x;

    char_to_uint(OutputPCData.name_5, "gy");
    OutputPCData.type_5 = 1;
    OutputPCData.data_5 = accel->y;

    char_to_uint(OutputPCData.name_6, "gz");
    OutputPCData.type_6 = 1;
    OutputPCData.data_6 = accel->z;

    char_to_uint(OutputPCData.name_7, "ax");
    OutputPCData.type_7 = 1;
    OutputPCData.data_7 = accel_point[0];

    char_to_uint(OutputPCData.name_8, "ay");
    OutputPCData.type_8 = 1;
    OutputPCData.data_8 = accel_point[1];

    char_to_uint(OutputPCData.name_9, "az");
    OutputPCData.type_9 = 1;
    OutputPCData.data_9 = accel_point[2];

    append_CRC16_check_sum((uint8_t *)&OutputPCData, sizeof(OutputPCData_s));
    memcpy(usb_tx_buf, &OutputPCData, sizeof(OutputPCData_s));
    CDC_Transmit_FS(usb_tx_buf, sizeof(OutputPCData_s));
}

/**
 * @brief      接收导航数据
 * @param[in]  none
 * @retval     None
 */
static void usb_receive_navigation(void)
{
    uint8_t crc_ok = verify_CRC16_check_sum((uint8_t *)usb_rx_buf, sizeof(ReceivedPacketTwist_s));
    if (crc_ok)
    {
        memcpy(&ReceivedPacketTwist, usb_rx_buf, sizeof(ReceivedPacketTwist_s));
    }
}

/**
 * @brief      接收视觉数据
 * @param[in]  none
 * @retval     None
 */
static void usb_receive_vision(void)
{
    uint8_t crc_ok = verify_CRC16_check_sum((uint8_t *)usb_rx_buf, sizeof(ReceivedPacketVision_s));
    if (crc_ok)
    {
        memcpy(&ReceivedPacketVision, usb_rx_buf, sizeof(ReceivedPacketVision_s));
        // buzzer_on(500, 30000);
    }
}

/**
 * @brief      接收LJW的串口调试助手的数据
 * @param[in]  none
 * @retval     None
 */
static void usb_receive_PC(void)
{
    uint8_t crc_ok = verify_CRC16_check_sum((uint8_t *)usb_rx_buf, sizeof(InputPCData_s));
    if (crc_ok)
    {
        memcpy(&InputPCData, usb_rx_buf, sizeof(InputPCData_s));
        // buzzer_on(500, 30000);
    }
}

/**
 * @brief      将字符串转换为uint8_t数组
 * @param[out] word: 转换后的数组
 * @param[in]  str: 原始字符串
 * @retval     None
 */
void char_to_uint(uint8_t *word, const char *str)
{
    int i = 0;
    while (str[i] != '\0' && i < 10)
    {
        word[i] = str[i];
        i++;
    }
}

/**
 * @brief          获取视觉接收数据
 * @param[in]      none
 * @return         视觉接收数据指针
 */
const ReceivedPacketVision_s *GetReceivedPacketVisionPoint(void)
{
    return &ReceivedPacketVision;
}

/**
 * @brief          获取视觉发送数据
 * @param[in]      none
 * @return         视觉发送数据指针
 * @note           用于修改需要发送的视觉数据
 */
SendPacketVision_s *GetSendPacketVisionPoint(void)
{
    return &SendPacketVision;
}

/**
 * @brief          获取导航接收数据
 * @param[in]      none
 * @return         导航接收数据指针
 */
const ReceivedPacketTwist_s *GetReceivedPacketTwistPoint(void)
{
    return &ReceivedPacketTwist;
}
