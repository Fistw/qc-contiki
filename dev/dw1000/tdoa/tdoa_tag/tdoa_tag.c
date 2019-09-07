/*
 * tdoa_tag
 * 
 * 实现TDOA算法中Tag功能
 * 
 */

#include "tdoa_tag.h"
// 含TDoA3帧结构体
#include "../tdoa.h"
// 含acket_t
#include "../mac.h"
// 含dwDevice_t dwTime_t定义以及decadriver接口
#include "../tdoa_decadriver.h"
// tag的util模块
#include "tdoa_tag_engine.h"
// #include "tdoa_tag_state.h"
#include "tdoa_tag_storage.h"

#include "contiki.h"
#include "estimator_kalman.h"

#include "fang.h"
#include "uwb_tdoa_anchor3.h"

#include <math.h>

static point_t tagCrd = {0,0,0,AGV_Z_AXIS_CONFIG};
static tdoaEngineState_t engineState;

#define PREAMBLE_LENGTH_S ( 128 * 1017.63e-9 ) //0.000130257
#define PREAMBLE_LENGTH (uint64_t)( PREAMBLE_LENGTH_S * 499.2e6 * 128 ) //832310.96832

#define TDMA_GUARD_LENGTH_S ( 1e-6 ) //0.000001
#define TDMA_GUARD_LENGTH (uint64_t)( TDMA_GUARD_LENGTH_S * 499.2e6 * 128 )

#define TDMA_EXTRA_LENGTH_S ( 300e-6 )
#define TDMA_EXTRA_LENGTH (uint64_t)( TDMA_EXTRA_LENGTH_S * 499.2e6 * 128 )

static bool isValidTimeStamp(const int64_t anchorRxTime)
{
  return anchorRxTime != 0;
}

static int updateRemoteData(tdoaAnchorContext_t *anchorCtx, const void *payload)
{
  const rangePacket3_t *packet = (rangePacket3_t *)payload;
  const void *anchorDataPtr = &packet->remoteAnchorData;
//  printf("remoteCount=%d\n",packet->header.remoteCount);
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

static inline void createVector(point_t* pstart, point_t* pend, point_t* vector)
{
    vector->x = pend->x - pstart->x;
    vector->y = pend->y - pstart->y;
    vector->z = pend->z - pstart->z;
}

static inline float multipleVector(point_t* v1, point_t* v2)
{
    return (v1->x)*(v2->x)+(v1->y)*(v2->y)+(v1->z)*(v2->z);
}

void correctTimestamp(int64_t* ptime, float fpp, float rxp, tdoaAnchorContext_t* anchorCtx)
{
  float rx_fp = rxp - fpp;
  float clk = anchorCtx->anchorInfo->clockCorrectionStorage.clockCorrection;
  point_t tmp;
  createVector(&tagCrd, &anchorCtx->anchorInfo->position, &tmp);
  float dis = sqrtf(multipleVector(&tmp, &tmp));

  if(rxp <= -86.218){ // node#0
    if(fpp <= -96.212){ // node#1
      if(dis <= 8.968){ // node#5
        if(clk <= 1.0){ // node#7
          if(rxp <= -89.747){ // node#9
            if(dis <= 8.652){ // node#23
              *ptime += 46; // node#93
            }else{
              *ptime += 95; // node#94
            }
          }else{
            if(dis <= 8.818){ // node#24
              if(rxp <= -88.545){ // node#69
                *ptime += 187; // node#97
              }else{
                *ptime += 147; // node#98
              }
            }else{
              *ptime += 220; // node#70
            }
          } 
        }else{
          if(dis <= 8.33){ // node#10
            *ptime += 327; // node#79
          }else{
            *ptime += 390; // node#80
          }
        } 
      }else{
        if(fpp <= -103.733){ // node#8
          if(rxp <= -89.347){ // node#13
            if(clk <= 1.0){ // node#15
              if(rxp <= -93.502){ // node#35
                if(clk <= 1.0){ // node#49
                  *ptime += 267; // node#59
                }else{
                  if(dis <= 11.765){ // node#60
                    *ptime += 251; // node#61
                  }else{
                    *ptime += 320; // node#62
                  }
                }
              }else{
                if(dis <= 11.108){ // node#50
                  if(rx_fp <= 18.502){ // node#51
                    *ptime += 316; // node#55
                  }else{
                    *ptime += 254; // node#56
                  }
                }else{
                  if(rxp <= -91.566){ // node#52
                    if(fpp <= -110.77){ // node#67
                      *ptime += 365; // node#85
                    }else{
                      *ptime += 433; // node#86
                    }
                  }else{
                    *ptime += 338; // node#68
                  }
                }
              }
            }else{
              if(dis <= 14.275){ // node#36
                if(rx_fp <= 22.003){ // node#39
                  if(rxp <= -94.831){ // node#63
                    *ptime += 294; // node#87
                  }else{
                    *ptime += 252; // node#88
                  }
                }else{
                  *ptime += 201; // node#64
                }
              }else{
                if(clk <= 1.0){ // node#40
                  *ptime += 336; // node#41
                }else{
                  if(dis <= 15.476){ // node#42
                    *ptime += 201; // node#95
                  }else{
                    *ptime += 254; // node#96
                  }
                }
              }
            }
          }else{
            if(dis <= 10.465){ // node#16
              if(rxp <= -87.766){ // node#43
                if(fpp <= -112.49){ // node#47
                  *ptime += 321; // node#73
                }else{
                  *ptime += 391; // node#74
                }
              }else{
                *ptime += 452; // node#48
              }
            }else{
              if(dis <= 11.363){ // node#44
                *ptime += 246; // node#45
              }else{
                if(rx_fp <= 25.708){ // node#46
                  *ptime += 429; // node#57
                }else{
                  *ptime += 339; // node#58
                }
              }
            }
          }
        }else{
          if(fpp <= -99.081){ // node#14
            if(dis <= 11.765){ // node#27
              if(dis <= 10.322){ // node#33
                *ptime += 291; // node#81
              }else{
                if(dis <= 10.89){ // node#82
                  *ptime += 206; // node#91
                }else{
                  *ptime += 241; // node#92
                }
              }
            }else{
              *ptime += 348; // node#34
            }
          }else{
            *ptime += 152; // node#28
          }
        }
      }
    }else{
      *ptime += 637; // node#6
    }
  }else{
    if(clk <= 1.0){ // node#2
      if(dis <= 4.924){ // node#3
        if(dis <= 3.885){ // node#11
          *ptime += 136; // node#37
        }else{
          if(rxp <= -79.643){ // node#38
            *ptime += 201; // node#71
          }else{ 
            *ptime += 251; // node#72
          }
        }
      }else{
        if(rxp <= -79.543){ // node#12
          if(fpp <= -95.688){ // node#17
            if(rxp <= -83.664){ // node#19
              if(dis <= 7.357){ // node#21
                *ptime += 99; // node#53
              }else{
                *ptime += 185; // node#54
              }
            }else{
              if(rx_fp <= 16.34){ // node#22
                *ptime += 31; // node#29
              }else{
                if(dis <= 8.335){ // node#30
                  if(rxp <= -81.596){ // node#31
                    if(dis <= 6.411){ // node#75
                      if(rx_fp <= 22.158){ // node#77
                        *ptime += 115; // node#83
                      }else{
                        *ptime += 57; // node#84
                      }
                    }else{
                      *ptime += 153; // node#78
                    }
                  }else{
                    *ptime += 160; // node#76
                  }
                }else{
                  *ptime += 58; // node#32
                }
              }
            }
          }else{
            *ptime += 214; // node#20
          }
        }else{
          *ptime += 13; // node#18
        }
      }
    }else{
      if(dis <= 4.016){ // node#4
        *ptime += 121; // node#25
      }else{
        if(rx_fp <= 12.786){ // node#26
          if(rx_fp <= 3.223){ // node#65
            *ptime += -20.862; // node#89
          }else{
            *ptime += 6.109; // node#90
          }
        }else{
          *ptime += 59; // node#66
        }
      }
    }
  }
}

void handleTdoaPacket(float fp_power, float rx_power, uint64_t rxTime, const packet_t *prxPacket)
{
  const uint8_t anchorId = prxPacket->sourceAddress[0];

  int64_t rxAn_by_T_in_cl_T = rxTime;

  const rangePacket3_t *packet = (rangePacket3_t *)prxPacket->payload;

  // uint32_t remoteTx = packet->header.txTimeStamp;
  const int64_t txAn_in_cl_An = packet->header.txTimeStamp;
  uint8_t seqNr = packet->header.seq;
  int8_t* tmp = packet->header.anchorCoordinate;
//  for(int i = 0; i < 6; i++)
//    printf("tmp[%d]=%d ",i,tmp[i]);
  //统计收包率
  printf("Debug: get packet from %d, seqNr=%d, now is %u\n", anchorId, seqNr, clock_time());
  printf("Debug: get packet from %d, txTimestamp=%llu\n", anchorId, txAn_in_cl_An);
  // process anchor packet in tag
  tdoaAnchorContext_t anchorCtx;
  uint32_t now_ms = clock_time();
  // using tdoa_storage to Get AnchorCtx for packet processing
  bool anchorCtxExist = tdoaEngineGetAnchorCtxForPacketProcessing(&engineState, anchorId, now_ms, &anchorCtx);
  tdoaStorageSetAnchorPosition(&anchorCtx, tmp[0]+tmp[1]*1e-2,tmp[2]+tmp[3]*1e-2,tmp[4]+tmp[5]*1e-2);
  
  // 决策树回归校正时间戳
  if(!tagCrd.timestamp && anchorCtxExist)
    correctTimestamp(&rxAn_by_T_in_cl_T, fp_power, rx_power, &anchorCtx);
  printf("After Correction(rxTimestamp): %llu\n", rxAn_by_T_in_cl_T);

  // // 粗暴设置基站位置
  // if (anchorId == 1)
  // {
  //    tdoaStorageSetAnchorPosition(&anchorCtx, 0.0, 0.0, 2.473);
  // }
  // else if(anchorId == 2)
  // {
  //    tdoaStorageSetAnchorPosition(&anchorCtx, 5.808, 0.0, 0.328);
  // }
  // else if(anchorId == 3)
  // {
	//  tdoaStorageSetAnchorPosition(&anchorCtx, 5.808, 0.0, 2.453);
  // }
  // else if(anchorId == 10)
  // {
	//  tdoaStorageSetAnchorPosition(&anchorCtx, 0.0, 0.0, 0.291);
  // }
  // else if(anchorId == 11)
  // {
	//  tdoaStorageSetAnchorPosition(&anchorCtx, 1.333, 5.633, 0.332);
  // }
  // else if(anchorId == 12)
  // {
  // 	 tdoaStorageSetAnchorPosition(&anchorCtx, 4.393, 5.633, 0.348);
  // }
  // else
  // {
	//  tdoaStorageSetAnchorPosition(&anchorCtx, 2.873, 5.633, 2.545);
  // }

  // 更新数据
  int rangeDataLength = updateRemoteData(&anchorCtx, packet);
//  uint32_t haha = clock_time();
//  for(int i = 0; i < 16; i++){
//    if(engineState.anchorInfoArray[i].isInitialized){
//    	printf("id=%d,seqNr=%d,",engineState.anchorInfoArray[i].id,engineState.anchorInfoArray[i].seqNr);
//    	for(int j = 0; j < 16; j++){
//    		if(engineState.anchorInfoArray[i].tof[j].endOfLife > haha)
//    			printf("tof:id=%d,tof=%lld,",engineState.anchorInfoArray[i].tof[j].id,engineState.anchorInfoArray[i].tof[j].tof);
//    		if(engineState.anchorInfoArray[i].remoteAnchorData[j].endOfLife > haha)
//    			printf("remoteData:id=%d,seqnr=%d,",engineState.anchorInfoArray[i].remoteAnchorData[j].id,engineState.anchorInfoArray[i].remoteAnchorData[j].seqNr);
//    	}
//    }
//  }
  // 计算位置
  tdoaEngineProcessPacket(&engineState, &anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T);
  // 设置rxtime和xtime
  tdoaStorageSetRxTxData(&anchorCtx, rxAn_by_T_in_cl_T, txAn_in_cl_An, seqNr);

  // rangingOk = true;
}

void handleAvoidPacket(uint32_t rxTime, const packet_t *prxPacket)
{

#ifdef UWB_TYPE_TAG_CONFIG
  int8_t* coor = &prxPacket->payload[1];
  point_t personCoor = {
    .x = coor[0]+coor[1]*1e-2,
    .y = coor[2]+coor[3]*1e-2,
  };
  uint8_t diff = (uint8_t)sqrtf(powf(tagCrd.x-personCoor.x,2)+powf(tagCrd.y-personCoor.y,2));
  if(diff <= 0x03){
    printf("AGV need stop.");
  }else if(diff <= 0x05){
    printf("AGV need slow down.");
  }
#endif

}

void handleTagRxPacket(float fp_power, float rx_power, uint64_t rxTime, const uint8_t *packetbuf, const uint16_t data_len)
{
  int dataLength = data_len;

  static packet_t rxPacket;
  uint8_t *prxPacket = &rxPacket;
  for (int i = 0; i < data_len; i++)
  {
    prxPacket[i] = packetbuf[i];
  }

  if (rxPacket.payload[0] == PACKET_TYPE_TDOA3)
  {
    handleTdoaPacket(fp_power, rx_power, rxTime, &rxPacket);
  }else if(rxPacket.payload[0] == PACKET_TYPE_AVOID){
    handleAvoidPacket(rxTime, &rxPacket);
  }else{
    printf("handleTagRxPacket: packet type don't known.");
  }
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
  printf("sendTdoaToEstimatorCallback is called\n");
  // estimatorKalman(&state, &sensors, clock_time(), tdoaMeasurement);
  // printf("Tag position is (%d, %d, %d)(mm)!\n", (int)(state.position.x*1000),
	// 	  	  	  	  	  	  	  	  	    (int)(state.position.y*1000),
	// 										(int)(state.position.z*1000));
//  if(fang(tdoaMeasurement, &tagCrd)){
//    printf("Executed fang successed and taylor successed.\n");
//  }else{
//    printf("Executed fang failed and taylor successed.\n");
//    // printf("sendTdoaToEstimatorCallback: calculate tag coodination fault.\n");
//  }
  fang_2D(tdoaMeasurement, &tagCrd);
  tagCrd.timestamp = clock_time();
  printf("The Coordinate of Tag is (%f, %f, %f) in timestamp: %u\n", tagCrd.x, tagCrd.y, tagCrd.z, tagCrd.timestamp);
// 人员安全避让功能
#ifdef UWB_TYPE_PERSON_CONFIG
      setupTx((float*)&tagCrd);
#endif
  // if(idx != -1){
  //   fangPutMatrix(queue, idx);
  //   if(calcTagCoordinate(&Am, (float*)b, &tagCrd))
  //     printf("The Coordinate of Tag is (%f, %f, %f) in timestamp: %u\n", tagCrd.x, tagCrd.y, tagCrd.z, tagCrd.timestamp);
  //   else
  //     printf("arm_matrix_inverse_f32 execute error.\n");
  // }
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
