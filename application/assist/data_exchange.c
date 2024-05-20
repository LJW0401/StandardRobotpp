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
#include "data_exchange.h"

#include "string.h"

typedef struct {
    uint8_t data[8];
} Data_t;

static Data_t DATA_BUFFER[Data_Exchange_INDEX_NUM] = {0};

/**
 * @brief          发布数据
 * @param[in]      index 数据索引
 * @param[in]      data 发布的数据（统一存储为8个字节）
 * @retval         none
 */
void Publish(DataExchangeIndex_e index, uint8_t data[8]) { memcpy(&DATA_BUFFER[index], data, 8); }

/**
 * @brief          订阅数据
 * @param[in]      index 数据索引
 * @retval         订阅数据的起始地址，需使用memcpy将值拷贝出来
 */
const uint8_t* Subscribe(DataExchangeIndex_e index) {}