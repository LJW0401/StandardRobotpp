/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       data_exchange.c/h
  * @brief      数据交换中心，用于各个模块之间的数据交换，不用相互调用头文件，加强各模块之间的解耦
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     May-22-2024     Penguin         1. 完成。
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */
#include "data_exchange.h"

#include "string.h"

#define DATA_LEN 4
typedef struct
{
    uint8_t data[DATA_LEN];
    DataType_e data_type;
} Data_t;

static Data_t DATA_BUFFER[Data_Exchange_INDEX_NUM] = {0};

/**
 * @brief          发布数据
 * @param[in]      index 数据索引
 * @param[in]      data 发布的数据（统一存储为4个字节）
 * @retval         none
 */
void Publish(DataExchangeIndex_e index, uint8_t * data, DataType_e data_type)
{
    memcpy(&DATA_BUFFER[index], data, DATA_LEN);
    DATA_BUFFER[index].data_type = data_type;
}

/**
 * @brief          订阅数据
 * @param[in]      index 数据索引
 * @param[in]      out 输出数据的地址
 * @retval         订阅数据的起始地址，需使用memcpy将值拷贝出来
 */
void Subscribe(DataExchangeIndex_e index, uint8_t * out)
{
    uint8_t data_len = 0;
    switch (DATA_BUFFER[index].data_type) {
        case DE_INT8:
        case DE_UINT8: {
            data_len = 1;
        } break;
        case DE_INT16:
        case DE_UINT16: {
            data_len = 2;
        } break;
        case DE_INT32:
        case DE_UINT32:
        case DE_FLOAT: {
            data_len = 4;
        } break;
        default:
            break;
    }

    memcpy(out, &DATA_BUFFER[index + DATA_LEN - data_len], data_len);
}