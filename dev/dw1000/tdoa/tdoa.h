/*
 * tdoa
 * 
 * TDoA3协议：https://wiki.bitcraze.io/doc:lps:tdoa3:protocol，一种无需基站间时钟同步的算法
 * 本文件包含算法实现需要用到的定义
 * 
 */
#ifndef __TDOA_H__
#define __TDOA_H__

#include <stdint.h>

#define PACKET_TYPE_TDOA3 0x30
#define PACKET_TYPE_AVOID 0x01

#define TDOA3_RECEIVE_TIMEOUT 10000

// TDoA Anchor protocol V3的头部定义
typedef struct
{
    uint8_t type;
    uint8_t seq;
    uint64_t txTimeStamp;
    int8_t anchorCoordinate[6];
    uint8_t remoteCount;
} __attribute__((packed)) rangePacketHeader3_t;

// 两种remote Anchor Data定义：是否包含distance
// 含有distance
typedef struct
{
    uint8_t id;
    uint8_t seq;
    uint64_t rxTimeStamp;
    uint16_t distance;
} __attribute__((packed)) remoteAnchorDataFull_t;
// 不含distance
typedef struct
{
    uint8_t id;
    uint8_t seq;
    uint64_t rxTimeStamp;
} __attribute__((packed)) remoteAnchorDataShort_t;

// TDoA Anchor protocol V3的Packet定义
// 包含头部header和remote Anchor Data
typedef struct
{
    rangePacketHeader3_t header;
    uint8_t remoteAnchorData;
} __attribute__((packed)) rangePacket3_t;

// 定义人员安全避让功能的帧负载格式
typedef struct 
{
    uint8_t type;
    int8_t personCoordinate[4];
} __attribute__((packed)) avoidPacket_t;

#endif
