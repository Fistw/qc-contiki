#include <string.h>

#include "tdoa_tag_storage.h"

// All times in milli seconds
#define TOF_VALIDITY_PERIOD (2 * 1000)
#define REMOTE_DATA_VALIDITY_PERIOD 30
#define ANCHOR_POSITION_VALIDITY_PERIOD (2 * 1000)
#define ANCHOR_ACTIVE_VALIDITY_PERIOD (2 * 1000)

void tdoaStorageInitialize(tdoaAnchorInfo_t anchorStorage[])
{
    memset(anchorStorage, 0, sizeof(tdoaAnchorInfo_t) * ANCHOR_STORAGE_COUNT);
}

static tdoaAnchorInfo_t *initializeSlot(tdoaAnchorInfo_t anchorStorage[], const uint8_t slot, const uint8_t anchor)
{
    memset(&anchorStorage[slot], 0, sizeof(tdoaAnchorInfo_t));
    anchorStorage[slot].id = anchor;
    anchorStorage[slot].isInitialized = true;

    return &anchorStorage[slot];
}

bool tdoaStorageGetCreateAnchorCtx(tdoaAnchorInfo_t anchorStorage[], const uint8_t anchor, const uint32_t currentTime_ms, tdoaAnchorContext_t *anchorCtx)
{
    anchorCtx->currentTime_ms = currentTime_ms;
    uint32_t oldestUpdateTime = currentTime_ms;
    int firstUninitializedSlot = -1;
    int oldestSlot = 0;

    for (int i = 0; i < ANCHOR_STORAGE_COUNT; i++)
    {
        if (anchorStorage[i].isInitialized)
        {
            if (anchor == anchorStorage[i].id)
            {
                anchorCtx->anchorInfo = &anchorStorage[i];
                return true;
            }

            if (anchorStorage[i].lastUpdateTime < oldestUpdateTime)
            {
                oldestUpdateTime = anchorStorage[i].lastUpdateTime;
                oldestSlot = i;
            }
        }
        else
        {
            if (firstUninitializedSlot == -1)
            {
                firstUninitializedSlot = i;
            }
        }
    }

    // The anchor was not found in storage
    tdoaAnchorInfo_t *newAnchorInfo = 0;
    if (firstUninitializedSlot != -1)
    {
        newAnchorInfo = initializeSlot(anchorStorage, firstUninitializedSlot, anchor);
    }
    else
    {
        newAnchorInfo = initializeSlot(anchorStorage, oldestSlot, anchor);
    }

    anchorCtx->anchorInfo = newAnchorInfo;
    return false;
}

bool tdoaStorageGetAnchorCtx(tdoaAnchorInfo_t anchorStorage[], const uint8_t anchor, const uint32_t currentTime_ms, tdoaAnchorContext_t *anchorCtx)
{
    anchorCtx->currentTime_ms = currentTime_ms;

    for (int i = 0; i < ANCHOR_STORAGE_COUNT; i++)
    {
        if (anchorStorage[i].isInitialized)
        {
            if (anchor == anchorStorage[i].id)
            {
                anchorCtx->anchorInfo = &anchorStorage[i];
                return true;
            }
        }
    }

    anchorCtx->anchorInfo = 0;
    return false;
}

uint8_t tdoaStorageGetId(const tdoaAnchorContext_t *anchorCtx)
{
    return anchorCtx->anchorInfo->id;
}

int64_t tdoaStorageGetRxTime(const tdoaAnchorContext_t *anchorCtx)
{
    return anchorCtx->anchorInfo->rxTime;
}

int64_t tdoaStorageGetTxTime(const tdoaAnchorContext_t *anchorCtx)
{
    return anchorCtx->anchorInfo->txTime;
}

uint8_t tdoaStorageGetSeqNr(const tdoaAnchorContext_t *anchorCtx)
{
    return anchorCtx->anchorInfo->seqNr;
}

uint32_t tdoaStorageGetLastUpdateTime(const tdoaAnchorContext_t *anchorCtx)
{
    return anchorCtx->anchorInfo->lastUpdateTime;
}

clockCorrectionStorage_t *tdoaStorageGetClockCorrectionStorage(const tdoaAnchorContext_t *anchorCtx)
{
    return &anchorCtx->anchorInfo->clockCorrectionStorage;
}

bool tdoaStorageGetAnchorPosition(const tdoaAnchorContext_t *anchorCtx, point_t *position)
{
    uint32_t now = anchorCtx->currentTime_ms;

    int32_t validCreationTime = now - ANCHOR_POSITION_VALIDITY_PERIOD;
    const tdoaAnchorInfo_t *anchorInfo = anchorCtx->anchorInfo;
    if ((int32_t)anchorInfo->position.timestamp > validCreationTime)
    {
        position->timestamp = anchorInfo->position.timestamp;
        position->x = anchorInfo->position.x;
        position->y = anchorInfo->position.y;
        position->z = anchorInfo->position.z;
        return true;
    }

    return false;
}

void tdoaStorageSetAnchorPosition(tdoaAnchorContext_t *anchorCtx, const float x, const float y, const float z)
{
    uint32_t now = anchorCtx->currentTime_ms;
    tdoaAnchorInfo_t *anchorInfo = anchorCtx->anchorInfo;

    anchorInfo->position.timestamp = now;
    anchorInfo->position.x = x;
    anchorInfo->position.y = y;
    anchorInfo->position.z = z;
}

void tdoaStorageSetRxTxData(tdoaAnchorContext_t *anchorCtx, int64_t rxTime, int64_t txTime, uint8_t seqNr)
{
    uint32_t now = anchorCtx->currentTime_ms;
    tdoaAnchorInfo_t *anchorInfo = anchorCtx->anchorInfo;

    anchorInfo->rxTime = rxTime;
    anchorInfo->txTime = txTime;
    anchorInfo->seqNr = seqNr;
    anchorInfo->lastUpdateTime = now;
}

double tdoaStorageGetClockCorrection(const tdoaAnchorContext_t *anchorCtx)
{
    return clockCorrectionEngineGet(&anchorCtx->anchorInfo->clockCorrectionStorage);
}

int64_t tdoaStorageGetRemoteRxTime(const tdoaAnchorContext_t *anchorCtx, const uint8_t remoteAnchor)
{
    const tdoaAnchorInfo_t *anchorInfo = anchorCtx->anchorInfo;

    for (int i = 0; i < REMOTE_ANCHOR_DATA_COUNT; i++)
    {
        if (remoteAnchor == anchorInfo->remoteAnchorData[i].id)
        {
            uint32_t now = anchorCtx->currentTime_ms;
            if (anchorInfo->remoteAnchorData[i].endOfLife > now)
            {
                return anchorInfo->remoteAnchorData[i].rxTime;
            }
            break;
        }
    }

    return 0;
}

void tdoaStorageSetRemoteRxTime(tdoaAnchorContext_t *anchorCtx, const uint8_t remoteAnchor, const int64_t remoteRxTime, const uint8_t remoteSeqNr)
{
    tdoaAnchorInfo_t *anchorInfo = anchorCtx->anchorInfo;

    int indexToUpdate = 0;
    uint32_t now = anchorCtx->currentTime_ms;
    uint32_t oldestTime = 0xFFFFFFFF;

    for (int i = 0; i < REMOTE_ANCHOR_DATA_COUNT; i++)
    {
        if (remoteAnchor == anchorInfo->remoteAnchorData[i].id)
        {
            indexToUpdate = i;
            break;
        }

        if (anchorInfo->remoteAnchorData[i].endOfLife < oldestTime)
        {
            oldestTime = anchorInfo->remoteAnchorData[i].endOfLife;
            indexToUpdate = i;
        }
    }

    anchorInfo->remoteAnchorData[indexToUpdate].id = remoteAnchor;
    anchorInfo->remoteAnchorData[indexToUpdate].rxTime = remoteRxTime;
    anchorInfo->remoteAnchorData[indexToUpdate].seqNr = remoteSeqNr;
    anchorInfo->remoteAnchorData[indexToUpdate].endOfLife = now + REMOTE_DATA_VALIDITY_PERIOD;
}

void tdoaStorageGetRemoteSeqNrList(const tdoaAnchorContext_t *anchorCtx, int *remoteCount, uint8_t seqNr[], uint8_t id[])
{
    const tdoaAnchorInfo_t *anchorInfo = anchorCtx->anchorInfo;
    uint32_t now = anchorCtx->currentTime_ms;

    int count = 0;

    for (int i = 0; i < REMOTE_ANCHOR_DATA_COUNT; i++)
    {
        if (anchorInfo->remoteAnchorData[i].endOfLife > now)
        {
            id[count] = anchorInfo->remoteAnchorData[i].id;
            seqNr[count] = anchorInfo->remoteAnchorData[i].seqNr;
            count++;
        }
    }

    *remoteCount = count;
}

int64_t tdoaStorageGetTimeOfFlight(const tdoaAnchorContext_t *anchorCtx, const uint8_t otherAnchor)
{
    const tdoaAnchorInfo_t *anchorInfo = anchorCtx->anchorInfo;

    for (int i = 0; i < TOF_PER_ANCHOR_COUNT; i++)
    {
        if (otherAnchor == anchorInfo->tof[i].id)
        {
            uint32_t now = anchorCtx->currentTime_ms;
            if (anchorInfo->tof[i].endOfLife > now)
            {
                return anchorInfo->tof[i].tof;
            }
            break;
        }
    }

    return 0;
}

void tdoaStorageSetTimeOfFlight(tdoaAnchorContext_t *anchorCtx, const uint8_t remoteAnchor, const int64_t tof)
{
    tdoaAnchorInfo_t *anchorInfo = anchorCtx->anchorInfo;

    int indexToUpdate = 0;
    uint32_t now = anchorCtx->currentTime_ms;
    uint32_t oldestTime = 0xFFFFFFFF;

    for (int i = 0; i < TOF_PER_ANCHOR_COUNT; i++)
    {
        if (remoteAnchor == anchorInfo->tof[i].id)
        {
            indexToUpdate = i;
            break;
        }

        if (anchorInfo->tof[i].endOfLife < oldestTime)
        {
            oldestTime = anchorInfo->tof[i].endOfLife;
            indexToUpdate = i;
        }
    }

    anchorInfo->tof[indexToUpdate].id = remoteAnchor;
    anchorInfo->tof[indexToUpdate].tof = tof;
    anchorInfo->tof[indexToUpdate].endOfLife = now + TOF_VALIDITY_PERIOD;
}

void setAnchorPosition(point_t* src, point_t* dst)
{
    dst->x = src->x;
    dst->y = src->y;
    dst->z = src->z;
}