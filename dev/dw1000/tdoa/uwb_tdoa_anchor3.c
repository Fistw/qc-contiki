/*
* @Author: craig
* @Date:   2018-12-28 20:10:46
* @Last Modified by:   craig
* @Last Modified time: 2019-01-04 11:29:07
*
* anchor的主要功能是互相双向测距
*/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "uwb.h"
#include "mac.h"
#include "uwb_tdoa_anchor3.h"
#include "decadriver/deca_regs.h"
#include "decadriver/deca_device_api.h"

// 根据设定的协议，定义数据包结构体及相关变量
// Time length of the preamble
// preamble是啥？
#define PREAMBLE_LENGTH_S ( 128 * 1017.63e-9 ) //0.000130257
#define PREAMBLE_LENGTH (uint64_t)( PREAMBLE_LENGTH_S * 499.2e6 * 128 ) //832310.96832

// Guard length to account for clock drift and time of flight
// 考虑了时钟漂移和飞行时间的保护长度？
#define TDMA_GUARD_LENGTH_S ( 1e-6 ) //0.000001
#define TDMA_GUARD_LENGTH (uint64_t)( TDMA_GUARD_LENGTH_S * 499.2e6 * 128 )

#define TDMA_EXTRA_LENGTH_S ( 300e-6 )
#define TDMA_EXTRA_LENGTH (uint64_t)( TDMA_EXTRA_LENGTH_S * 499.2e6 * 128 )

#define TDMA_HIGH_RES_RAND_S ( 1e-3 )
#define TDMA_HIGH_RES_RAND (uint64_t)( TDMA_HIGH_RES_RAND_S * 499.2e6 * 128 )

#define ANCHOR_LIST_UPDATE_INTERVAL 1000; //Anchor列表更新间隔

#define CLOCK_CORRECTION_BUCKET_MAX 4
#define CLOCK_CORRECTION_ACCEPTED_NOISE 0.03e-6

#define MAX_CLOCK_DEVIATION_SPEC 10e-6
#define CLOCK_CORRECTION_FILTER 0.1d

#define CLOCK_CORRECTION_SPEC_MIN (1.0d - MAX_CLOCK_DEVIATION_SPEC * 2)
#define CLOCK_CORRECTION_SPEC_MAX (1.0d + MAX_CLOCK_DEVIATION_SPEC * 2)

#define ANCHOR_STORAGE_COUNT 16
#define REMOTE_TX_MAX_COUNT 8 // REMOTE_TX_MAX_COUNT < ANCHOR_STORAGE_COUNT

#define ANTENNA_OFFSET 154.6   // In meters
#define ANTENNA_DELAY  ((ANTENNA_OFFSET*499.2e6*128)/299792458.0) // In radio tick
#define MIN_TOF ANTENNA_DELAY

#define ID_COUNT 256 //用8位整型来存储id，最多也就256个
#define ID_WITHOUT_CONTEXT 0xff

#define SYSTEM_TX_FREQ 400.0
#define ANCHOR_MAX_TX_FREQ 50.0
// We need a lower limit of minimum tx rate. The TX timestamp in the protocol is
// only 32 bits (equal to 67 ms) and we want to avoid double wraps of the TX counter.
// To have some margin set the lowest tx frequency to 20 Hz (= 50 ms)
#define ANCHOR_MIN_TX_FREQ 20.0

////////////////////////////////// FreeRTOSConfig.h中定义M2T()，应该是跟时间有关的单位转换
#define DISTANCE_VALIDITY_PERIOD (2 * 1000);

// Useful constants
static const uint8_t base_address[] = {0, 0, 0, 0, 0, 0, 0xcf, 0xbc};

// 表示本地存储的Remote anchor context信息的结构体
typedef struct
{
    uint8_t id;

    bool isUsed; // 一个标识位：这个anchorCtx是否是有用的

    // 协议内容
    uint8_t seqNr;
    uint32_t rxTimeStamp;
    uint32_t txTimeStamp; //???
    uint16_t distance;
    uint32_t distanceUpdateTime;
    bool isDataGoodForTransmission; //???

    // 维护时钟偏移率/
    double clockCorrection;
    int clockCorrectionBucket;
} anchorContext_t;

//整个数据包的内容及本算法用到的全部变量
static struct ctx_s
{
    int anchorId;

    // Header部分的数据
    uint8_t seqNr;   // 最近发送的包的序列号
    uint32_t txTime; // 最近发送的包的发送时间戳，由UWB标记

    uint32_t nextTxTick; // 下一次发送包的发送时间(system clock ticks)
    int averageTxDelay;  // ms

    // List of ids to transmit in remote data section
    uint8_t remoteTxId[REMOTE_TX_MAX_COUNT];
    uint8_t remoteTxIdCount;

    // 要定期更新要传输和存储的anchor列表
    uint32_t nextAnchorListUpdate; //下次列表更新的时间点

    // Remote anchor data
    uint8_t anchorCtxLookup[ID_COUNT];               //用来标记各个Anchor有没有Context 信息存储，以及存储对应于anchorCtx的第几个空间中
    anchorContext_t anchorCtx[ANCHOR_STORAGE_COUNT]; //存储在本地的remote anchor context信息
    uint8_t anchorRxCount[ID_COUNT];                 //这个列表储存收到其他Anchor发送包的次数，比如收到过id为a的Anchor发送的数据，anchorRxCount[a]就为收到过的次数
} ctx;                                               // 定义了一个ctx_s的结构体变量 ctx

// Packet formats
#define PACKET_TYPE_TDOA3 0x30

// https://wiki.bitcraze.io/doc:lps:tdoa3:protocol
// TDoA Anchor protocol V3的头部定义
typedef struct
{
    uint8_t type;
    uint8_t seq;
    uint32_t txTimeStamp;
    uint8_t remoteCount;
} __attribute__((packed)) rangePacketHeader3_t;

// 两种remote Anchor Data定义：是否包含distance
typedef struct
{
    uint8_t id;
    uint8_t seq;
    uint32_t rxTimeStamp;
    uint16_t distance;
} __attribute__((packed)) remoteAnchorDataFull_t;

typedef struct
{
    uint8_t id;
    uint8_t seq;
    uint32_t rxTimeStamp;
} __attribute__((packed)) remoteAnchorDataShort_t;

// packet的定义？
typedef struct
{
    rangePacketHeader3_t header;
    uint8_t remoteAnchorData; //data部分的定义？
} __attribute__((packed)) rangePacket3_t;

// 重置AnchorRxCount列表
static void clearAnchorRxCount()
{
    memset(&ctx.anchorRxCount, 0, ID_COUNT);
}

// 获取Context
static anchorContext_t *getContext(uint8_t anchorId)
{
    uint8_t slot = ctx.anchorCtxLookup[anchorId];

    if (slot == ID_WITHOUT_CONTEXT)
    {
        return 0;
    }

    return &ctx.anchorCtx[slot];
}

// 更新ctx.anchorCtx，更新anchorCtxLookup列表：移除
static void removeAnchorContextsNotInList(const uint8_t *id, const uint8_t count)
{
    for (int i = 0; i < ANCHOR_STORAGE_COUNT; i++)
    {
        anchorContext_t *anchorCtx = &ctx.anchorCtx[i];
        if (anchorCtx->isUsed)
        {
            const uint8_t ctxId = anchorCtx->id;
            bool found = false;
            for (int j = 0; j < count; j++)
            {
                if (id[j] == ctxId)
                {
                    found = true;
                    break;
                }
            }

            if (!found)
            {
                ctx.anchorCtxLookup[ctxId] = ID_WITHOUT_CONTEXT;
                anchorCtx->isUsed = false;
            }
        }
    }
}

// 更新ctx.anchorCtx，更新anchorCtxLookup列表：添加
static void createAnchorContext(const uint8_t id)
{
    if (ctx.anchorCtxLookup[id] != ID_WITHOUT_CONTEXT)
    {
        // Already has a context, we're done
        return;
    }

    for (uint8_t i = 0; i < ANCHOR_STORAGE_COUNT; i++)
    {
        anchorContext_t *anchorCtx = &ctx.anchorCtx[i];
        // 把没有使用的anchorCtx空间用来创建新的
        if (!anchorCtx->isUsed)
        {
            ctx.anchorCtxLookup[id] = i;

            memset(anchorCtx, 0, sizeof(anchorContext_t));
            anchorCtx->id = id;
            anchorCtx->isUsed = true;

            break;
        }
    }
}

static void createAnchorContextsInList(const uint8_t *id, const uint8_t count)
{
    for (uint8_t i = 0; i < count; i++)
    {
        createAnchorContext(id[i]);
    }
}

// 清除无效数据
static void purgeData()
{
    ////////////////////////////////// xTaskGetTickCount需更换成contiki平台系统时间获取接口
    uint32_t now = clock_time();
    uint32_t acceptedCreationTime = now - DISTANCE_VALIDITY_PERIOD;

    for (int i = 0; i < ANCHOR_STORAGE_COUNT; i++)
    {
        anchorContext_t *anchorCtx = &ctx.anchorCtx[i];
        if (anchorCtx->isUsed)
        {
            if (anchorCtx->distanceUpdateTime < acceptedCreationTime)
            {
                anchorCtx->distance = 0;

                anchorCtx->clockCorrection = 0.0;
                anchorCtx->clockCorrectionBucket = 0;
            }
        }
    }
}

// 更新AnchorList
// 作用：更新要存储的Remote Anchor data列表，并添加到要发送的消息中
static void updateAnchorLists()
{
    // Randomize which anchors to use

    static uint8_t availableId[ID_COUNT]; // 用来统计收到过数据的Anchor的id，也即周边看见看见的Anchor有哪些
    static bool availableUsed[ID_COUNT];  // 临时变量，下面循环中表示该id是否已被添加过
    memset(availableId, 0, sizeof(availableId));
    memset(availableUsed, 0, sizeof(availableUsed));
    int availableCount = 0; // 临时变量，用来记录上面两个列表中实际有多少个id

    static uint8_t ctxts[ANCHOR_STORAGE_COUNT];
    memset(ctxts, 0, sizeof(ctxts));

    // Collect all anchors we have got a message from
    // 首先遍历接收列表anchorRxCount，为availableId填值，这些id的anchor data将被存储或准备发送
    for (int i = 0; i < ID_COUNT; i++)
    {
        if (ctx.anchorRxCount[i] != 0)
        {
            availableId[availableCount++] = i; // 存储id
        }
    }

    // Out of all anchors that we have received messages from, pick two randomized subsets for storage and TX ids
    uint8_t remoteTXIdIndex = 0; // 临时变量
    uint8_t contextIndex = 0;    //临时变量

    // 总共挑选ANCHOR_STORAGE_COUNT个id，每次挑选一个
    for (int i = 0; i < ANCHOR_STORAGE_COUNT; i++)
    {
        // 在一个随机起始位置start开始遍历一遍availableId
        int start = rand() % availableCount;
        // Scan forward until we find an anchor
        for (int j = start; j < (start + availableCount); j++)
        {
            const int index = j % availableCount;
            if (!availableUsed[index])
            {

                const int id = availableId[index];
                if (remoteTXIdIndex < REMOTE_TX_MAX_COUNT)
                {
                    // 更新要发送的remote anchor data id列表
                    ctx.remoteTxId[remoteTXIdIndex++] = id;
                }
                if (contextIndex < ANCHOR_STORAGE_COUNT)
                {
                    // 更新要存储的id列表
                    ctxts[contextIndex++] = id;
                }

                availableUsed[index] = true;
                break;
            }
        }
    }

    // 根据选取的要存储的Anchor data列表，标记数据清除或创建
    removeAnchorContextsNotInList(ctxts, contextIndex);
    createAnchorContextsInList(ctxts, contextIndex);

    ctx.remoteTxIdCount = remoteTXIdIndex;

    clearAnchorRxCount();

    // Set the TX rate based on the number of transmitting anchors around us
    // Aim for 400 messages/s. Up to 8 anchors: 50 Hz / anchor
    // SYSTEM_TX_FREQ 400.0
    // ANCHOR_MAX_TX_FREQ 50.0
    // ANCHOR_MIN_TX_FREQ 20.0
    float freq = SYSTEM_TX_FREQ / (availableCount + 1);
    if (freq > ANCHOR_MAX_TX_FREQ)
    {
        freq = ANCHOR_MAX_TX_FREQ;
    }
    if (freq < ANCHOR_MIN_TX_FREQ)
    {
        freq = ANCHOR_MIN_TX_FREQ;
    }
    ctx.averageTxDelay = 1000.0 / freq;

    purgeData();
}

static double calculateClockCorrection(anchorContext_t *anchorCtx, int remoteTxSeqNr, uint32_t remoteTx, uint32_t rx)
{
    double result = 0.0d;

    // Assigning to uint32_t truncates the diffs and takes care of wrapping clocks
    uint32_t tickCountRemote = remoteTx - anchorCtx->txTimeStamp;
    uint32_t tickCountLocal = rx - anchorCtx->rxTimeStamp;

    if (tickCountRemote != 0)
    {
        result = (double)tickCountLocal / (double)tickCountRemote;
    }

    return result;
}

static void fillClockCorrectionBucket(anchorContext_t *anchorCtx)
{
    if (anchorCtx->clockCorrectionBucket < CLOCK_CORRECTION_BUCKET_MAX)
    {
        anchorCtx->clockCorrectionBucket++;
    }
}

static bool emptyClockCorrectionBucket(anchorContext_t *anchorCtx)
{
    if (anchorCtx->clockCorrectionBucket > 0)
    {
        anchorCtx->clockCorrectionBucket--;
        return false;
    }

    return true;
}

static bool updateClockCorrection(anchorContext_t *anchorCtx, double clockCorrection)
{
    const double diff = clockCorrection - anchorCtx->clockCorrection;
    bool sampleIsAccepted = false;

    if (-CLOCK_CORRECTION_ACCEPTED_NOISE < diff && diff < CLOCK_CORRECTION_ACCEPTED_NOISE)
    {
        // LP filter
        anchorCtx->clockCorrection = anchorCtx->clockCorrection * (1.0d - CLOCK_CORRECTION_FILTER) + clockCorrection * CLOCK_CORRECTION_FILTER;

        fillClockCorrectionBucket(anchorCtx);
        sampleIsAccepted = true;
    }
    else
    {
        if (emptyClockCorrectionBucket(anchorCtx))
        {
            if (CLOCK_CORRECTION_SPEC_MIN < clockCorrection && clockCorrection < CLOCK_CORRECTION_SPEC_MAX)
            {
                anchorCtx->clockCorrection = clockCorrection;
            }
        }
    }

    return sampleIsAccepted;
}

static bool extractFromPacket(const rangePacket3_t *rangePacket, uint32_t *remoteRx, uint8_t *remoteRxSeqNr)
{
    const void *anchorDataPtr = &rangePacket->remoteAnchorData;
    //chenxin?这个循环什么意思，难道有多个remote date section段？
    for (uint8_t i = 0; i < rangePacket->header.remoteCount; i++)
    {
        remoteAnchorDataFull_t *anchorData = (remoteAnchorDataFull_t *)anchorDataPtr;

        const uint8_t id = anchorData->id;
        if (id == ctx.anchorId)
        {
            *remoteRxSeqNr = anchorData->seq & 0x7f; //chenxin:这是作为发包方的远端基站收到的此基站上一次包的序列号，seq第一位是hasDistance，后七位为序列号
            *remoteRx = anchorData->rxTimeStamp;     //chenxin:这是作为发包方的远端基站收到的此基站上一次包的时间
            return true;
        }

        bool hasDistance = ((anchorData->seq & 0x80) != 0);
        //chenxin?remote data section不止一段
        if (hasDistance)
        {
            anchorDataPtr += sizeof(remoteAnchorDataFull_t);
        }
        else
        {
            anchorDataPtr += sizeof(remoteAnchorDataShort_t);
        }
    }

    return false;
}

static uint16_t calculateDistance(anchorContext_t *anchorCtx, int remoteRxSeqNr, uint32_t remoteTx, uint32_t remoteRx, uint32_t rx)
{ //chenxin:好像是计算基站0，1之间的tof，不过这样的话clockCorrection也就是基站0，1之间的了
    // Check that the remote received seq nr is our latest tx seq nr
    if (remoteRxSeqNr == ctx.seqNr && anchorCtx->clockCorrection > 0.0d)
    {
        uint32_t localTime = rx - ctx.txTime;
        uint32_t remoteTime = (uint32_t)((double)(remoteTx - remoteRx) * anchorCtx->clockCorrection);
        uint32_t distance = (localTime - remoteTime) / 2;

        return distance & 0xfffful;
    }
    else
    {
        return 0;
    }
}

static void handleRangePacket(const uint32_t rxTime, const packet_t *rxPacket)
{
    const uint8_t remoteAnchorId = rxPacket->sourceAddress[0];
    printf("ID: %d\n", remoteAnchorId);

    const rangePacket3_t *rangePacket = (rangePacket3_t *)rxPacket->payload;
    uint32_t remoteTx = rangePacket->header.txTimeStamp;
    uint8_t remoteTxSeqNr = rangePacket->header.seq;
    printf("remote: %d %d\n", remoteTx, remoteTxSeqNr);

    ctx.anchorRxCount[remoteAnchorId]++;
    anchorContext_t *anchorCtx = getContext(remoteAnchorId);
    if (anchorCtx)
    {
        const rangePacket3_t *rangePacket = (rangePacket3_t *)rxPacket->payload;

        uint32_t remoteTx = rangePacket->header.txTimeStamp;
        uint8_t remoteTxSeqNr = rangePacket->header.seq;
        printf("remote: %d %d\n", remoteTx, remoteTxSeqNr);

        double clockCorrection = calculateClockCorrection(anchorCtx, remoteTxSeqNr, remoteTx, rxTime);
        if (updateClockCorrection(anchorCtx, clockCorrection))
        {
            anchorCtx->isDataGoodForTransmission = true;

            uint32_t remoteRx = 0;
            uint8_t remoteRxSeqNr = 0;
            bool dataFound = extractFromPacket(rangePacket, &remoteRx, &remoteRxSeqNr);
            if (dataFound)
            {
                uint16_t distance = calculateDistance(anchorCtx, remoteRxSeqNr, remoteTx, remoteRx, rxTime);

                // TODO krri Remove outliers in distances
                if (distance > MIN_TOF)
                {
                    anchorCtx->distance = distance;
                    //   anchorCtx->distanceUpdateTime = xTaskGetTickCount();
                    anchorCtx->distanceUpdateTime = clock_time();
                }
            }
        }
        else
        {
            anchorCtx->isDataGoodForTransmission = false;
        }
        //chenxin:利用收到目的基站的最新包更新目的基站的上下文
        anchorCtx->rxTimeStamp = rxTime;
        anchorCtx->seqNr = remoteTxSeqNr;
        anchorCtx->txTimeStamp = remoteTx;
    }
}

void handleRxPacket(const uint8_t *packetbuf, const uint16_t data_len)
{
    static packet_t rxPacket;
    uint8_t *prxPacket = &rxPacket;
    int dataLength = data_len;
    for (int i = 0; i < data_len; i++)
    {
    	printf("%d ", packetbuf[i]);
        prxPacket[i] = packetbuf[i];
    }
    printf("\n");
    //   dwTime_t rxTime = { .full = 0 };

    //   dwGetRawReceiveTimestamp(dev, &rxTime);
    // dwCorrectTimestamp(dev, &rxTime); //api
    if (rxPacket.payload[0] != PACKET_TYPE_TDOA3)
    {
    	printf("Not a TDOA3 Packet.\n");
    	return;
    }
    handleRangePacket(rxTime.low32, &rxPacket);
    //   rxPacket.payload[0] = 0;

    //   dwGetData(dev, (uint8_t*)&rxPacket, dataLength);

    //   if (dataLength == 0) {
    //     return;
    //   }

    //   switch(rxPacket.payload[0]) {
    //   case PACKET_TYPE_TDOA3:
    //     handleRangePacket(rxTime.low32, &rxPacket);
    //     break;
    //   case SHORT_LPP:
    //     if (rxPacket.destAddress[0] == ctx.anchorId) {
    //       lppHandleShortPacket(&rxPacket.payload[1], dataLength - MAC802154_HEADER_LENGTH - 1);
    //     }
    //     break;
    //   default:
    //     // Do nothing
    //     break;
    //   }
}

// Adjust time for schedule transfer by DW1000 radio. Set 9 LSB to 0 and round up
static void adjustTxRxTime(dwTime_t *time)
{
    time->full = (time->full & ~((1 << 9) - 1)) + (1 << 9);
}

////////////////////////////////// dwTime_t ？？
static dwTime_t findTransmitTimeAsSoonAsPossible(dwDevice_t *dev)
{
    dwTime_t transmitTime = {.full = 0};
    //////////////////////////////////
    dwGetSystemTimestamp(&transmitTime);
    printf("before: %u %u\n", transmitTime.high8, transmitTime.low32);
    printf("before: %u %u\n", transmitTime.high32, transmitTime.low8);
    // Add guard and preamble time
    transmitTime.full += TDMA_GUARD_LENGTH*10;
    transmitTime.full += PREAMBLE_LENGTH*10;

    // And some extra
    transmitTime.full += TDMA_EXTRA_LENGTH*10;

    // TODO krri Adding randomization on this level adds a long delay, is it worth it?
    // The randomization on OS level is quantized to 1 ms (tick time of the system)
    // Add a high res random to smooth it out
    // uint32_t r = rand();
    // uint32_t delay = r % TDMA_HIGH_RES_RAND;
    // transmitTime.full += delay;

    // DW1000 can only schedule time with 9 LSB at 0, adjust for it
    adjustTxRxTime(&transmitTime);

    printf("after: %u %u\n", transmitTime.high8, transmitTime.low32);
    printf("after: %u %u\n", transmitTime.high32, transmitTime.low8);
    return transmitTime;
}

// 填充TX 数据
static int populateTxData(rangePacket3_t *rangePacket)
{
    // rangePacket->header.type already populated
    rangePacket->header.seq = ctx.seqNr;
    rangePacket->header.txTimeStamp = ctx.txTime;

    uint8_t remoteAnchorCount = 0;
    uint8_t *anchorDataPtr = &rangePacket->remoteAnchorData;
    for (uint8_t i = 0; i < ctx.remoteTxIdCount; i++)
    {
        remoteAnchorDataFull_t *anchorData = (remoteAnchorDataFull_t *)anchorDataPtr;

        uint8_t id = ctx.remoteTxId[i];
        anchorContext_t *anchorCtx = getContext(id);

        if (anchorCtx->isDataGoodForTransmission)
        {
            anchorData->id = id;
            anchorData->seq = anchorCtx->seqNr;
            anchorData->rxTimeStamp = anchorCtx->rxTimeStamp;

            if (anchorCtx->distance > 0)
            {
                anchorData->distance = anchorCtx->distance;
                anchorDataPtr += sizeof(remoteAnchorDataFull_t);
                anchorData->seq |= 0x80;
            }
            else
            {
                anchorDataPtr += sizeof(remoteAnchorDataShort_t);
            }

            remoteAnchorCount++;
        }
    }
    rangePacket->header.remoteCount = remoteAnchorCount;

    return (uint8_t *)anchorDataPtr - (uint8_t *)rangePacket;
}

// Set TX data in the radio TX buffer
////////////////////////////////// dwDevice_t
static void setTxData(dwDevice_t *dev)
{
    ////////////////////////////////// packet_t在mac.h中定义
    static packet_t txPacket;
    static bool firstEntry = true;
    static int lppLength = 0;

    if (firstEntry)
    {

        memcpy(txPacket.sourceAddress, base_address, 8);
        txPacket.sourceAddress[0] = ctx.anchorId;
        memcpy(txPacket.destAddress, base_address, 8);
        txPacket.destAddress[0] = 0xff;

        txPacket.payload[0] = PACKET_TYPE_TDOA3;

        firstEntry = false;
    }

    uwbConfig_t *uwbConfig = uwbGetConfig();

    int rangePacketSize = populateTxData((rangePacket3_t *)txPacket.payload);

    printf("send:\n");
    for (int i = 0; i < rangePacketSize; i++)
    {
    	printf("%d ", txPacket.payload[i]);
    }
    printf("\n");

    // LPP anchor position is currently sent in all packets
    // if (uwbConfig->positionEnabled)
    // {
    //     txPacket.payload[rangePacketSize + LPP_HEADER] = SHORT_LPP;
    //     txPacket.payload[rangePacketSize + LPP_TYPE] = LPP_SHORT_ANCHOR_POSITION;

    //     struct lppShortAnchorPosition_s *pos = (struct lppShortAnchorPosition_s *) &txPacket.payload[rangePacketSize + LPP_PAYLOAD];
    // memcpy(pos->position, uwbConfig->position, 3 * sizeof(float));

    //     lppLength = 2 + sizeof(struct lppShortAnchorPosition_s);
    // }

    // dwSetData(dev, (uint8_t *)&txPacket, MAC802154_HEADER_LENGTH + rangePacketSize + lppLength);
    dwSetData(dev, (uint8_t *)&txPacket, MAC802154_HEADER_LENGTH + rangePacketSize);
}

// Setup the radio to send a packet
static void setupTx(dwDevice_t *dev)
{
    dwTime_t txTime = findTransmitTimeAsSoonAsPossible(dev);
    ctx.txTime = txTime.low32;
    ctx.seqNr = (ctx.seqNr + 1) & 0x7f;

    setTxData(dev);

    //////////////////////////////////
    // dwNewTransmit(dev);
    // dev->deviceMode = TX_MODE;
    // dwSetDefaults(dev);
    // dwSetTxRxTime(dev, txTime);

    int ret = dwStartTransmit(&txTime);
    printf("ret = %d\n", ret);
//    if (ret == 0)
//    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
//    {
////    	printf("%d\n", dwt_read32bitreg(SYS_STATUS_ID));
////    	clock_wait(1);
//    }
}

static uint32_t randomizeDelayToNextTx()
{
    const uint32_t interval = 10;

    uint32_t r = rand();
    // 产生随机发送延迟时长的方法
    uint32_t delay = ctx.averageTxDelay + r % interval - interval / 2;

//    return delay;
    return 3000;
}

// 数据初始化
// 参数uwbConfig_t见uwb.h中定义
void tdoa3Init(uwbConfig_t *config)
{
    ctx.anchorId = config->address[0];
    ctx.seqNr = 0;
    ctx.txTime = 0;
    ctx.nextTxTick = 0;
    ctx.averageTxDelay = 1000.0 / ANCHOR_MAX_TX_FREQ;

    ctx.remoteTxIdCount = 0;
    ctx.nextAnchorListUpdate = 0;

    // ID_COUNT 256
    // ID_WITHOUT_CONTEXT 0xff
    memset(&ctx.anchorCtxLookup, ID_WITHOUT_CONTEXT, ID_COUNT); //初始化anchorCtxLookup

    for (int i = 0; i < ANCHOR_STORAGE_COUNT; i++)
    {
        ctx.anchorCtx[i].isUsed = false;
    }

    clearAnchorRxCount();

    srand(ctx.anchorId); //用来提供随机数种子
}

// 有一些uwb事件中断发生，比如收到包，发送包，超时等；
void tdoa3UwbEvent(dwDevice_t *dev)
{
	printf("tdoa3uwbevent is called.\n");
    uint32_t now = clock_time();
    if (now > ctx.nextAnchorListUpdate)
    {
        updateAnchorLists();
        ctx.nextAnchorListUpdate = now + ANCHOR_LIST_UPDATE_INTERVAL; // 更新update时间
    }

//    if (ctx.nextTxTick < now)
//    {
//        uint32_t newDelay = randomizeDelayToNextTx();
//        ////////////////////////////////// M2T()方法
//        ctx.nextTxTick = now + newDelay;
//
//        // 准备发送
//        printf("setupTx is called.\n");
//        setupTx(dev);
//    }

    setupTx(dev);
//
//    uint32_t timeout_ms =  ctx.nextTxTick - now;
//    return timeout_ms;
}
