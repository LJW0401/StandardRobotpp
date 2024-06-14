#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#include "struct_typedef.h"

#define FRAME_HEADER_SOF 0xA5
#define FRAME_HEADER_LEN 4  // （字节）数据帧头部长度

typedef struct
{
    struct frame_header
    {
        uint8_t sof;  // 数据帧起始字节，固定值为 0xA5
        uint8_t len;  // 数据段长度
        uint8_t id;   // 数据段id
        uint8_t crc;  // 数据帧头的 CRC8 校验
    } __attribute__((packed)) frame_header;

    uint8_t data[30];
    uint16_t crc;
} __attribute__((packed)) BoardCommunicateData_s;

extern BoardCommunicateData_s BOARD_TX_DATA;

extern void Usart1Init(void);

extern void DataPack(uint8_t * data, uint16_t data_lenth);

extern void DataUnpack(void);

#endif  // __COMMUNICATION_H
