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

#ifndef USB_TASK_H
#define USB_TASK_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
  uint8_t header;
  bool tracking : 1;
  uint8_t id : 3;         // 0-outpost 6-guard 7-base
  uint8_t armors_num : 3; // 2-balance 3-outpost 4-normal
  uint8_t reserved : 1;
  float x;
  float y;
  float z;
  float yaw;
  float vx;
  float vy;
  float vz;
  float v_yaw;
  float r1;
  float r2;
  float dz;
  uint16_t checksum;
} __attribute__((packed)) ReceivedPacketVision_s;

typedef struct
{
  uint8_t header;
  uint8_t detect_color : 1; // 0-red 1-blue
  bool reset_tracker : 1;
  uint8_t reserved : 6;
  float roll;
  float pitch;
  float yaw;
  float aim_x;
  float aim_y;
  float aim_z;
  uint16_t checksum;
} __attribute__((packed)) SendPacketVision_s;

typedef struct
{
  uint8_t header;
  float linear_x;
  float linear_y;
  float linear_z;
  float angular_x;
  float angular_y;
  float angular_z;
  uint16_t checksum;
} __attribute__((packed)) ReceivedPacketTwist_s;

typedef struct
{
  uint8_t header;
  uint16_t red_1_robot_hp;
  uint16_t red_2_robot_hp;
  uint16_t red_3_robot_hp;
  uint16_t red_4_robot_hp;
  uint16_t red_5_robot_hp;
  uint16_t red_7_robot_hp;
  uint16_t red_outpost_hp;
  uint16_t red_base_hp;
  uint16_t blue_1_robot_hp;
  uint16_t blue_2_robot_hp;
  uint16_t blue_3_robot_hp;
  uint16_t blue_4_robot_hp;
  uint16_t blue_5_robot_hp;
  uint16_t blue_7_robot_hp;
  uint16_t blue_outpost_hp;
  uint16_t blue_base_hp;
  uint16_t checksum;
} __attribute__((packed)) SendPacketAllRobotHP_s;

typedef struct
{
  uint8_t header;
  uint8_t game_progress;
  uint16_t stage_remain_time;
  uint16_t checksum;
} __attribute__((packed)) SendPacketGameStatus_s;

typedef struct
{
  uint8_t header;
  uint8_t robot_id;
  uint16_t current_hp;
  uint16_t shooter_heat;
  bool team_color; // 0-red 1-blue
  bool is_attacked;
  uint16_t checksum;
} __attribute__((packed)) SendPacketRobotStatus_s;

typedef struct
{
  uint8_t header;
  uint16_t ecd_set;
  uint16_t checksum;
} __attribute__((packed)) InputPCData_s;

typedef struct
{
  uint8_t header;
  uint16_t length;
  uint8_t name_1[10];
  uint8_t type_1;
  float data_1;
  uint8_t name_2[10];
  uint8_t type_2;
  float data_2;
  uint8_t name_3[10];
  uint8_t type_3;
  float data_3;
  uint8_t name_4[10];
  uint8_t type_4;
  float data_4;
  uint8_t name_5[10];
  uint8_t type_5;
  float data_5;
  uint8_t name_6[10];
  uint8_t type_6;
  float data_6;
  uint8_t name_7[10];
  uint8_t type_7;
  float data_7;
  uint8_t name_8[10];
  uint8_t type_8;
  float data_8;
  uint8_t name_9[10];
  uint8_t type_9;
  float data_9;
  uint8_t name_10[10];
  uint8_t type_10;
  float data_10;
  uint16_t checksum;
} __attribute__((packed)) OutputPCData_s;


extern OutputPCData_s OutputPCData;


extern void usb_task(void const *argument);

const ReceivedPacketVision_s *GetReceivedPacketVisionPoint(void);
SendPacketVision_s *GetSendPacketVisionPoint(void);

const ReceivedPacketTwist_s *GetReceivedPacketTwistPoint(void);

#endif
