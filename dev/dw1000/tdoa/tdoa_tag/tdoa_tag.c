/*
 * tdoa_tag
 * 
 * 实现TDoA算法中Tag的功能
 * 
 */

#include "tdoa_tag.h"
// 含TDoA3帧结构体
#include "../tdoa.h"
// 含packet_t
#include "../mac.h"
// 含dwDevice_t dwTime_t定义以及decadriver接口
#include "../tdoa_decadriver.h"
// tag的util模块
#include "tdoa_tag_engine.h"
// #include "tdoa_tag_state.h"
#include "tdoa_tag_storage.h"

static tdoaEngineState_t engineState;

static bool isValidTimeStamp(const int64_t anchorRxTime)
{
  return anchorRxTime != 0;
}

static int updateRemoteData(tdoaAnchorContext_t *anchorCtx, const void *payload)
{
  const rangePacket3_t *packet = (rangePacket3_t *)payload;
  const void *anchorDataPtr = &packet->remoteAnchorData;
  for (uint8_t i = 0; i < packet->header.remoteCount; i++)
  {
    remoteAnchorDataFull_t *anchorData = (remoteAnchorDataFull_t *)anchorDataPtr;

    uint8_t remoteId = anchorData->id;
    int64_t remoteRxTime = anchorData->rxTimeStamp;
    uint8_t remoteSeqNr = anchorData->seq & 0x7f;

    if (isValidTimeStamp(remoteRxTime))
    {
      tdoaStorageSetRemoteRxTime(anchorCtx, remoteId, remoteRxTime, remoteSeqNr);
    }

    bool hasDistance = ((anchorData->seq & 0x80) != 0);
    if (hasDistance)
    {
      int64_t tof = anchorData->distance;
      if (isValidTimeStamp(tof))
      {
        tdoaStorageSetTimeOfFlight(anchorCtx, remoteId, tof);

        uint8_t anchorId = tdoaStorageGetId(anchorCtx);
        // tdoaStats_t *stats = &engineState.stats;
        // if (anchorId == stats->anchorId && remoteId == stats->remoteAnchorId)
        // {
        //   stats->tof = (uint16_t)tof;
        // }
      }

      anchorDataPtr += sizeof(remoteAnchorDataFull_t);
    }
    else
    {
      anchorDataPtr += sizeof(remoteAnchorDataShort_t);
    }
  }

  return (uint8_t *)anchorDataPtr - (uint8_t *)packet;
}

void handleTagRxPacket(uint32_t rxTime, const uint8_t *packetbuf, const uint16_t data_len)
{
  int dataLength = data_len;

  static packet_t rxPacket;
  uint8_t *prxPacket = &rxPacket;
  for (int i = 0; i < data_len; i++)
  {
    prxPacket[i] = packetbuf[i];
  }

  if (rxPacket.payload[0] != PACKET_TYPE_TDOA3)
  {
    printf("Not a TDOA3 Packet.\n");
    return;
  }

  const uint8_t anchorId = rxPacket.sourceAddress[0];

  dwTime_t arrival = {.full = 0};
  arrival.low32 = rxTime;
  const int64_t rxAn_by_T_in_cl_T = arrival.full;

  const rangePacket3_t *packet = (rangePacket3_t *)rxPacket.payload;

  // uint32_t remoteTx = packet->header.txTimeStamp;
  const int64_t txAn_in_cl_An = packet->header.txTimeStamp;
  uint8_t seqNr = packet->header.seq;

  // process anchor packet in tag
  tdoaAnchorContext_t anchorCtx;
  uint32_t now_ms = clock_time();
  // using tdoa_storage to Get AnchorCtx for packet processing
  tdoaEngineGetAnchorCtxForPacketProcessing(&engineState, anchorId, now_ms, &anchorCtx);
  // 更新数据
  int rangeDataLength = updateRemoteData(&anchorCtx, packet);
  // 计算位置
  tdoaEngineProcessPacket(&engineState, &anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T);
  // 设置rxtime和txtime
  tdoaStorageSetRxTxData(&anchorCtx, rxAn_by_T_in_cl_T, txAn_in_cl_An, seqNr);

  // 粗暴设置基站位置
  if (anchorId == 2)
  {
    tdoaStorageSetAnchorPosition(&anchorCtx, 0.0, 0.0, 0.0);
  }
  else{
	  tdoaStorageSetAnchorPosition(&anchorCtx, 1.0,1.0,1.0);
  }
  // rangingOk = true;
}

// // 获取基站位置
// static bool getAnchorPosition(const uint8_t anchorId, point_t *position)
// {
//   tdoaAnchorContext_t anchorCtx;
//   uint32_t now_ms = clock_time();

//   bool contextFound = tdoaStorageGetAnchorCtx(engineState.anchorInfoArray, anchorId, now_ms, &anchorCtx);
//   if (contextFound)
//   {
//     tdoaStorageGetAnchorPosition(&anchorCtx, position);
//     return true;
//   }

//   return false;
// }

// // 获取基站列表
// static uint8_t getAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize)
// {
//   return tdoaStorageGetListOfAnchorIds(engineState.anchorInfoArray, unorderedAnchorList, maxListSize);
// }

// static uint8_t getActiveAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize)
// {
//   uint32_t now_ms = T2M(xTaskGetTickCount());
//   return tdoaStorageGetListOfActiveAnchorIds(engineState.anchorInfoArray, unorderedAnchorList, maxListSize, now_ms);
// }

static void sendTdoaToEstimatorCallback(tdoaMeasurement_t *tdoaMeasurement)
{
  // kalman 估计方法暂不使用
  // estimatorKalmanEnqueueTDOA(tdoaMeasurement);
  printf("sendTdoaToEstimatorCallback is called\n");

#ifdef LPS_2D_POSITION_HEIGHT
  // If LPS_2D_POSITION_HEIGHT is defined we assume that we are doing 2D positioning.
  // LPS_2D_POSITION_HEIGHT contains the height (Z) that the tag will be located at
  heightMeasurement_t heightData;
  heightData.timestamp = xTaskGetTickCount();
  heightData.height = LPS_2D_POSITION_HEIGHT;
  heightData.stdDev = 0.0001;
  estimatorKalmanEnqueueAsoluteHeight(&heightData);
#endif
}

// 编译这部分忽略警告
//#pragma GCC diagnostic push
//#pragma GCC diagnostic ignored "-Wunused-parameter"
void tdoaTagInit()
{
  uint32_t now_ms = clock_time();
  tdoaEngineInit(&engineState, now_ms, sendTdoaToEstimatorCallback);
#ifdef LPS_2D_POSITION_HEIGHT
  DEBUG_PRINT("2D positioning enabled at %f m height\n", LPS_2D_POSITION_HEIGHT);
#endif

  /////////////////////
  // 设置接收超时
  // dwSetReceiveWaitTimeout(dev, TDOA3_RECEIVE_TIMEOUT);

  // dwCommitConfiguration(dev);

  // rangingOk = false;
}
//#pragma GCC diagnostic pop

static void onEvent()
{
  // eventPacketSent
  // eventPacketReceived
  // eventReceiveTimeout
  // eventTimeout

  // 更新状态
  // uint32_t now_ms = clock_time();
  // tdoaStatsUpdate(&engineState.stats, now_ms);
}
