#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#include "struct_typedef.h"

#define FRAME_HEADER_SOF 0xA5
#define FRAME_HEADER_LEN 5  // （字节）数据帧头部长度
#define DATA_LEN 20         // 数据段长度
#define DATA_NUM 8          // 数据段数量

#define FRAME_HEADER_LEN_OFFEST 1
#define FRAME_HEADER_ID_OFFEST 2
#define FRAME_HEADER_TYPE_OFFEST 3

typedef struct
{
    struct frame_header
    {
        uint8_t sof;   // 数据帧起始字节，固定值为 0xA5
        uint8_t len;   // 数据段长度
        uint8_t id;    // 数据段id
        uint8_t type;  // 数据段类型
        uint8_t crc;   // 数据帧头的 CRC8 校验
    } __attribute__((packed)) frame_header;

    uint8_t data[DATA_LEN];
    uint16_t crc;
} __attribute__((packed)) BoardCommunicateData_s;

extern BoardCommunicateData_s BOARD_TX_DATA;

extern void Usart1Init(void);

extern void DataPack(uint8_t * data, uint8_t data_lenth, uint8_t data_id);

extern void DataUnpack(void);

#endif  // __COMMUNICATION_H
