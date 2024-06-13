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

#define DATA_LIST_LEN 10
#define NAME_LEN 16

typedef struct
{
    void * data_address;
    uint32_t data_size;
    char data_name[NAME_LEN];
} Data_t;

static Data_t DATA_LIST[DATA_LIST_LEN] = {0};
static uint8_t USED_LEN = 0;  // 已经使用的数据量

/**
 * @brief          发布数据
 * @param[in]      address 数据地址
 * @param[in]      name 数据名称(最大长度为15字符)
 * @retval         数据发布状态
 */
uint8_t Publish(void * address, char * name)
{
    if (USED_LEN >= DATA_LIST_LEN) {  // 判断数据列表已满
        return PUBLISH_ALREADY_FULL;
    }

    for (uint8_t i = 0; i < USED_LEN; i++) {  // 判断数据是否已经存在
        if (DATA_LIST[i].data_address == address) {
            return PUBLISH_ALREADY_EXIST;
        }
    }

    // 保存数据
    // DATA_LIST[USED_LEN].data_address = address;
    memcpy(&DATA_LIST[USED_LEN].data_address, &address, 4);
    memcpy(DATA_LIST[USED_LEN].data_name, name, NAME_LEN);
    USED_LEN++;
    return PUBLISH_OK;
}

/**
 * @brief          订阅数据
 * @param[in]      name 数据名称
 * @retval         订阅数据的地址
 */
const void * Subscribe(char * name)
{
    for (uint8_t i = 0; i < USED_LEN; i++) {
        if (strcmp(DATA_LIST[i].data_name, name) == 0) {
            return DATA_LIST[i].data_address;
        }
    }

    return NULL;
}
