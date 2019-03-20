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

#define TRUNCATE_TO_ANCHOR_TS_BITMAP 0x00FFFFFFFF
uint64_t truncateToAnchorTimeStamp(uint64_t fullTimeStamp)
{
    return fullTimeStamp & TRUNCATE_TO_ANCHOR_TS_BITMAP;
}

static void enqueueTDOA(const tdoaAnchorContext_t *anchorACtx, const tdoaAnchorContext_t *anchorBCtx, const double distanceDiff, tdoaEngineState_t *engineState)
{
    tdoaMeasurement_t tdoa;
    point_t posA, posB;
    if (tdoaStorageGetAnchorPosition(anchorACtx, &posA) && tdoaStorageGetAnchorPosition(anchorBCtx, &posB))
    {
        uint8_t idA = tdoaStorageGetId(anchorACtx);
        uint8_t idB = tdoaStorageGetId(anchorBCtx);
        if (idA < idB)
        {
            tdoa.idA = idA;
            tdoa.idB = idB;
            setAnchorPosition(&posA, &tdoa.anchorPosition[0]);
            setAnchorPosition(&posB, &tdoa.anchorPosition[1]);
            tdoa.distanceDiff = -distanceDiff;
        }
        else
        {
            tdoa.idA = idB;
            tdoa.idB = idA;
            setAnchorPosition(&posB, &tdoa.anchorPosition[0]);
            setAnchorPosition(&posA, &tdoa.anchorPosition[1]);
            tdoa.distanceDiff = distanceDiff;
        }
        int64_t tof = tdoaStorageGetTimeOfFlight(anchorACtx, idB);
        tdoa.distance = SPEED_OF_LIGHT * tof / UWB_TS_FREQ;
        tdoa.endOfLife = clock_time() + TDOA_EXPIRED;

        printf("get the distance diff from  %d  and  %d  :::  %lf,\n the distance between them is %lf\n", idA, idB, distanceDiff,tdoa.distance);
        engineState->sendTdoaToEstimator(&tdoa);
    }
    // tdoaStats_t *stats = &engineState->stats;

    // tdoaMeasurement_t tdoa = {tdoa.stdDev = MEASUREMENT_NOISE_STD,
    // 						  tdoa.distanceDiff = distanceDiff};
    // if (tdoaStorageGetAnchorPosition(anchorACtx, &tdoa.anchorPosition[0]) && tdoaStorageGetAnchorPosition(anchorBCtx, &tdoa.anchorPosition[1]))
    // {e:sameSlot = 3, firstEmptySlot = 5, oldestSlot = 0␊
    //     // stats->packetsToEstimator++;
    //     engineState->sendTdoaToEstimator(&tdoa);

    //     uint8_t idA = tdoaStorageGetId(anchorACtx);
    //     uint8_t idB = tdoaStorageGetId(anchorBCtx);

    //     int diff1 = distanceDiff;
    //     int diff2 = (distanceDiff - diff1) * 1e9;
    //     printf("get the distance diff from  %d  and  %d  :::  %d.%09d\n", idA, idB, diff1, diff2);
    //     // if (idA == stats->anchorId && idB == stats->remoteAnchorId)
    //     // {
    //     //     stats->tdoa = distanceDiff;
    //     // }
    //     // if (idB == stats->anchorId && idA == stats->remoteAnchorId)
    //     // {
    //     //     stats->tdoa = -distanceDiff;
    //     // }
    // }
}

static bool updateClockCorrection(tdoaAnchorContext_t *anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T)
{
    bool sampleIsReliable = false;

    const int64_t latest_rxAn_by_T_in_cl_T = tdoaStorageGetRxTime(anchorCtx);
    const int64_t latest_txAn_in_cl_An = tdoaStorageGetTxTime(anchorCtx);

    if (latest_rxAn_by_T_in_cl_T != 0 && latest_txAn_in_cl_An != 0)
    {
        double clockCorrectionCandidate = clockCorrectionEngineCalculate(rxAn_by_T_in_cl_T, latest_rxAn_by_T_in_cl_T, txAn_in_cl_An, latest_txAn_in_cl_An, TRUNCATE_TO_ANCHOR_TS_BITMAP);
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

    return sampleIsReliable;
}

static int64_t calcTDoA(const tdoaAnchorContext_t *otherAnchorCtx, const tdoaAnchorContext_t *anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T)
{
    const uint8_t otherAnchorId = tdoaStorageGetId(otherAnchorCtx);

    const int64_t tof_Ar_to_An_in_cl_An = tdoaStorageGetTimeOfFlight(anchorCtx, otherAnchorId);
    const int64_t rxAr_by_An_in_cl_An = tdoaStorageGetRemoteRxTime(anchorCtx, otherAnchorId);
    const double clockCorrection = tdoaStorageGetClockCorrection(anchorCtx);

    const int64_t rxAr_by_T_in_cl_T = tdoaStorageGetRxTime(otherAnchorCtx);

    const int64_t delta_txAr_to_txAn_in_cl_An = (tof_Ar_to_An_in_cl_An + truncateToAnchorTimeStamp(txAn_in_cl_An - rxAr_by_An_in_cl_An));
    const int64_t timeDiffOfArrival_in_cl_T = truncateToAnchorTimeStamp(rxAn_by_T_in_cl_T - rxAr_by_T_in_cl_T) - delta_txAr_to_txAn_in_cl_An * clockCorrection;

    return timeDiffOfArrival_in_cl_T;
}

static double calcDistanceDiff(const tdoaAnchorContext_t *otherAnchorCtx, const tdoaAnchorContext_t *anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T, const double locodeckTsFreq)
{
    const int64_t tdoa = calcTDoA(otherAnchorCtx, anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T);
    return SPEED_OF_LIGHT * tdoa / locodeckTsFreq;
}

static bool findSuitableAnchor(tdoaEngineState_t *engineState, tdoaAnchorContext_t *otherAnchorCtx, const tdoaAnchorContext_t *anchorCtx)
{
    static uint8_t seqNr[REMOTE_ANCHOR_DATA_COUNT];
    static uint8_t id[REMOTE_ANCHOR_DATA_COUNT];
    static uint8_t offset = 0;

    if (tdoaStorageGetClockCorrection(anchorCtx) <= 0.0)
    {
        return false;
    }

    offset++;
    int remoteCount = 0;
    tdoaStorageGetRemoteSeqNrList(anchorCtx, &remoteCount, seqNr, id);

    uint32_t now_ms = anchorCtx->currentTime_ms;

    // Loop over the candidates and pick the first one that is useful
    // An offset (updated for each call) is added to make sure we start at
    // different positions in the list and vary which candidate to choose
    for (int i = offset; i < (remoteCount + offset); i++)
    {
        uint8_t index = i % remoteCount;
        const uint8_t candidateAnchorId = id[index];
        if (tdoaStorageGetCreateAnchorCtx(engineState->anchorInfoArray, candidateAnchorId, now_ms, otherAnchorCtx))
        {
            if (seqNr[index] == tdoaStorageGetSeqNr(otherAnchorCtx) && tdoaStorageGetTimeOfFlight(anchorCtx, candidateAnchorId))
            {
                return true;
            }
        }
    }

    otherAnchorCtx->anchorInfo = 0;
    return false;
}

void tdoaEngineGetAnchorCtxForPacketProcessing(tdoaEngineState_t *engineState, const uint8_t anchorId, const uint32_t currentTime_ms, tdoaAnchorContext_t *anchorCtx)
{
    tdoaStorageGetCreateAnchorCtx(engineState->anchorInfoArray, anchorId, currentTime_ms, anchorCtx);
    // if (tdoaStorageGetCreateAnchorCtx(engineState->anchorInfoArray, anchorId, currentTime_ms, anchorCtx))
    // {
    //     engineState->stats.contextHitCount++;
    // }
    // else
    // {
    //     engineState->stats.contextMissCount++;
    // }
}

void tdoaEngineProcessPacket(tdoaEngineState_t *engineState, tdoaAnchorContext_t *anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T)
{
    // bool timeIsGood = updateClockCorrection(anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T, &engineState->stats);
    bool timeIsGood = updateClockCorrection(anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T);

    if (timeIsGood)
    {
        // engineState->stats.timeIsGood++;
        printf("TimeIsGood\n");
        tdoaAnchorContext_t otherAnchorCtx;
        if (findSuitableAnchor(engineState, &otherAnchorCtx, anchorCtx))
        {
            printf("found suitable anchor\n");
            // engineState->stats.suitableDataFound++;
            // 计算距离差
            double tdoaDistDiff = calcDistanceDiff(&otherAnchorCtx, anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T, engineState->tsFreq);
            // 根据新的tdoa数据更新位置
            enqueueTDOA(&otherAnchorCtx, anchorCtx, tdoaDistDiff, engineState);
        }
    }
}
