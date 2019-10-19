/*
Implementation of LPS TDoA Tag functionality

The tag is assumed to move around in a large system of anchors. Any anchor ids
can be used, and the same anchor id can even be used by multiple anchors as long
as they are not visible in the same area. It is assumed that the anchor density
is evenly distributed in the covered volume and that 5-20 anchors are visible
in every point. The tag is attached to a physical object and the expected
velocity is a few m/s, this means that anchors are within range for a time
period of seconds.

假设标签在大型锚系统中移动。 可以使用任何锚定ID，并且只要在同一区域中不可见，多个锚点甚至可以使用相同的锚定ID。 假设锚密度
均匀分布在覆盖的体积中，每个点都可以看到5-20个锚。 标签附加到物理对象上，预期速度为几m / s，这意味着锚点在范围内的时间段为秒。

The implementation must handle
1. An infinite number of anchors, where around 20 are visible at one time
2. Any anchor ids
3. Dynamically changing visibility of anchors over time
4. Random TX times from anchors with possible packet collisions and packet loss

*/

#include <string.h>
#include <stdio.h>

#include "tdoa_tag_engine.h"

#include "agv_types.h"

#define MEASUREMENT_NOISE_STD 0.15f

void tdoaEngineInit(tdoaEngineState_t *engineState, const uint32_t now_ms, tdoaEngineSendTdoaToEstimator sendTdoaToEstimator)
{
    tdoaStorageInitialize(engineState->anchorInfoArray);
    // tdoaStatsInit(&engineState->stats, now_ms);
    engineState->sendTdoaToEstimator = sendTdoaToEstimator;
    engineState->tsFreq = UWB_TS_FREQ;
}

#define TRUNCATE_TO_ANCHOR_TS_BITMAP 0xFFFFFFFFFF
uint64_t truncateToAnchorTimeStamp(uint64_t fullTimeStamp)
{
    return fullTimeStamp & TRUNCATE_TO_ANCHOR_TS_BITMAP;
}

static void enqueueTDOA(int count, tdoaAnchorContext_t *anchorACtx, const tdoaAnchorContext_t *anchorBCtx, double* distanceDiff, tdoaEngineState_t *engineState)
{
    int i;
    tdoaMeasurement_t tdoas[count];
    for(i = 0; i < count; i++){
        point_t posA, posB;
        if (tdoaStorageGetAnchorPosition(&anchorACtx[i], &posA) && tdoaStorageGetAnchorPosition(anchorBCtx, &posB))
        {
            uint8_t idA = tdoaStorageGetId(&anchorACtx[i]);
            uint8_t idB = tdoaStorageGetId(anchorBCtx);
            tdoas[i].idA = idB;
            tdoas[i].idB = idA;
            setAnchorPosition(&posB, &tdoas[i].anchorPosition[0]);
            setAnchorPosition(&posA, &tdoas[i].anchorPosition[1]);
            tdoas[i].distanceDiff = -distanceDiff[i];
            printf("get the distance diff from  %d  and  %d  :::  %f\n", idB, idA, -distanceDiff[i]);
        }
        else
        {
            return;
        }
        
    }
    engineState->sendTdoaToEstimator((tdoaMeasurement_t*)tdoas, count);
}

static bool updateClockCorrection(tdoaAnchorContext_t *anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T)
{
    bool sampleIsReliable = false;

    const int64_t latest_rxAn_by_T_in_cl_T = tdoaStorageGetRxTime(anchorCtx);
    const int64_t latest_txAn_in_cl_An = tdoaStorageGetTxTime(anchorCtx);
    printf("rxAn_by_T_in_cl_T-latest_txAn_in_cl_An=%lf\n",(double)(rxAn_by_T_in_cl_T-latest_txAn_in_cl_An)/499.2e6/128);
    if (latest_rxAn_by_T_in_cl_T != 0 && latest_txAn_in_cl_An != 0)
    {
        double clockCorrectionCandidate = clockCorrectionEngineCalculate(rxAn_by_T_in_cl_T, latest_rxAn_by_T_in_cl_T, txAn_in_cl_An, latest_txAn_in_cl_An, TRUNCATE_TO_ANCHOR_TS_BITMAP);
        printf("clockCorrectionCandinate=%lf\n",clockCorrectionCandidate);
        sampleIsReliable = clockCorrectionEngineUpdate(tdoaStorageGetClockCorrectionStorage(anchorCtx), clockCorrectionCandidate);

        // if (sampleIsReliable)
        // {
        //     if (tdoaStorageGetId(anchorCtx) == stats->anchorId)
        //     {
        //         stats->clockCorrection = tdoaStorageGetClockCorrection(anchorCtx);
        //         stats->clockCorrectionCount++;
        //     }
        // }
    }
//    printf("sampleIsReliable=%d\n",sampleIsReliable);
    return sampleIsReliable;
}

static int64_t calcTDoA(int count, const double locodeckTsFreq, double* tdoaDistDiff, const tdoaAnchorContext_t *otherAnchorCtx, const tdoaAnchorContext_t *anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T)
{   
    int i;
    for(i = 0; i < count; i++){
        const uint8_t otherAnchorId = tdoaStorageGetId(otherAnchorCtx+i);

        const int64_t tof_Ar_to_An_in_cl_An = tdoaStorageGetTimeOfFlight(anchorCtx, otherAnchorId);
        const int64_t rxAr_by_An_in_cl_An = tdoaStorageGetRemoteRxTime(anchorCtx, otherAnchorId);
        const double clockCorrection = tdoaStorageGetClockCorrection(anchorCtx);

        const int64_t rxAr_by_T_in_cl_T = tdoaStorageGetRxTime(otherAnchorCtx+i);

        const int64_t delta_txAr_to_txAn_in_cl_An = (tof_Ar_to_An_in_cl_An + truncateToAnchorTimeStamp(txAn_in_cl_An - rxAr_by_An_in_cl_An));
        const int64_t timeDiffOfArrival_in_cl_T = truncateToAnchorTimeStamp(rxAn_by_T_in_cl_T - rxAr_by_T_in_cl_T) - delta_txAr_to_txAn_in_cl_An * clockCorrection;

        tdoaDistDiff[i] = SPEED_OF_LIGHT * timeDiffOfArrival_in_cl_T / locodeckTsFreq;
    }
    // return timeDiffOfArrival_in_cl_T;
}

static double calcDistanceDiff(int count, double* tdoaDistDiff, tdoaAnchorContext_t *otherAnchorCtx, const tdoaAnchorContext_t *anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T, const double locodeckTsFreq)
{
    // const int64_t tdoa = calcTDoA(otherAnchorCtx, anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T);
    // return SPEED_OF_LIGHT * tdoa / locodeckTsFreq;
    calcTDoA(count, locodeckTsFreq, tdoaDistDiff, otherAnchorCtx, anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T);
}

static int findSuitableAnchor(tdoaEngineState_t *engineState, tdoaAnchorContext_t* otherAnchorCtxs, const tdoaAnchorContext_t *anchorCtx)
{
    static uint8_t seqNr[REMOTE_ANCHOR_DATA_COUNT];
    static uint8_t id[REMOTE_ANCHOR_DATA_COUNT];
    static uint8_t offset = 0;

    if (tdoaStorageGetClockCorrection(anchorCtx) <= 0.0)
    {
        return false;
    }

    int remoteCount = 0;
    tdoaStorageGetRemoteSeqNrList(anchorCtx, &remoteCount, seqNr, id);
//    printf("remotecount=%d\n",remoteCount);

    uint32_t now_ms = anchorCtx->currentTime_ms;

    // generate random offset
//    offset++;
    srand(now_ms);
    offset = rand() % remoteCount;

    // Loop over the candidates and pick the first one that is useful
    // An offset (updated for each call) is added to make sure we start at
    // different positions in the list and vary which candidate to choose
    int count = 0;
    for (int i = offset; i < (remoteCount + offset); i++)
    {
        uint8_t index = i % remoteCount;
        const uint8_t candidateAnchorId = id[index];
        if (tdoaStorageGetCreateAnchorCtx(engineState->anchorInfoArray, candidateAnchorId, now_ms, &otherAnchorCtxs[count]))
        {
            if (seqNr[index] == tdoaStorageGetSeqNr(&otherAnchorCtxs[count]) && tdoaStorageGetTimeOfFlight(anchorCtx, candidateAnchorId))
            {
                //return true;
                count++;
            }
        }
    }
//    printf("count=%d\n",count);
    return count;
}

bool tdoaEngineGetAnchorCtxForPacketProcessing(tdoaEngineState_t *engineState, const uint8_t anchorId, const uint32_t currentTime_ms, tdoaAnchorContext_t *anchorCtx)
{
    return tdoaStorageGetCreateAnchorCtx(engineState->anchorInfoArray, anchorId, currentTime_ms, anchorCtx);
    // if (tdoaStorageGetCreateAnchorCtx(engineState->anchorInfoArray, anchorId, currentTime_ms, anchorCtx))
    // {
    //     engineState->stats.contextHitCount++;
    // }
    // else
    // {
    //     engineState->stats.contextMissCount++;
    // }
}

//　修改取数据计算ＴＤＯＡ逻辑：取出全部数据用于计算冗余ＴＤＯＡ．
void tdoaEngineProcessPacket(tdoaEngineState_t *engineState, tdoaAnchorContext_t *anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T)
{
    // bool timeIsGood = updateClockCorrection(anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T, &engineState->stats);
    bool timeIsGood = updateClockCorrection(anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T);

    if (timeIsGood)
    {
        // engineState->stats.timeIsGood++;
        printf("TimeIsGood\n");
//        tdoaAnchorContext_t otherAnchorCtxs[3];
        tdoaAnchorContext_t otherAnchorCtxs[REMOTE_ANCHOR_DATA_COUNT];
        int count;
        if ((count = findSuitableAnchor(engineState, otherAnchorCtxs, anchorCtx)) >= 3)
        {
//            printf("found suitable anchor\n");
            // engineState->stats.suitableDataFound++;
            // 计算距离差
            double tdoaDistDiffs[count];
            calcDistanceDiff(count, tdoaDistDiffs, otherAnchorCtxs, anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T, engineState->tsFreq);
            // 根据新的tdoa数据更新位置
            enqueueTDOA(count, otherAnchorCtxs, anchorCtx, tdoaDistDiffs, engineState);
        }else{
        	printf("Can't found suitable anchors\n");
        }
    }
}
