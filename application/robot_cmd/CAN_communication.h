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

#ifndef CAN_COMMUNICATION_H
#define CAN_COMMUNICATION_H

#include "CAN_cmd_cybergear.h"
#include "CAN_cmd_damiao.h"
#include "CAN_cmd_dji.h"
#include "CAN_cmd_lingkong.h"
#include "CAN_receive.h"

// clang-format off
#define BOARD_DATA_ANY     ((uint16_t)0xA00)
#define BOARD_DATA_UINT16  ((uint16_t)0xB00)
// clang-format on

#endif  // CAN_COMMUNICATION_H
