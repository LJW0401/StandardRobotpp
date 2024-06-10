/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       data_exchange.c/h
  * @brief      数据交换中心，用于各个模块之间的数据交换，不用相互调用头文件，加强各模块之间的解耦
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

#ifndef __DATA_EXCHANGE_H
#define __DATA_EXCHANGE_H
#include "struct_typedef.h"

typedef enum __DataExchangeIndex {
    TEST_DATA = 0,
    YAW_ANGLE,
    Data_Exchange_INDEX_NUM
} DataExchangeIndex_e;

typedef enum __Data_Type {
    DE_INT8 = 0,
    DE_UINT8,
    DE_INT16,
    DE_UINT16,
    DE_INT32,
    DE_UINT32,
    DE_FLOAT,
    Data_Type_NUM
} DataType_e;

typedef enum DataPublishStatus {
    PUBLISH_FAIL = 0,
    PUBLISH_OK,
    PUBLISH_ALREADY_EXIST,
    PUBLISH_ALREADY_FULL
} DataPublishStatus_e;

typedef enum DataSubscribeStatus { SUBSCRIBE_FAIL = 0, SUBSCRIBE_OK } DataSubscribeStatus_e;

typedef struct __Imu
{
    float yaw, pitch, roll;              // rad
    float yaw_vel, pitch_vel, roll_vel;  // rad/s
    float x_accel, y_accel, z_accel;     // m/s^2
} Imu_t;

extern uint8_t Publish(void * address, uint32_t size, char * name);
extern uint8_t Subscribe(void * out, char * name);

#endif  // __DATA_EXCHANGE_H
