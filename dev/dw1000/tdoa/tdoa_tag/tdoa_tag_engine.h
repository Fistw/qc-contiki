#ifndef __TDOA_TAG_ENGINE_H__
#define __TDOA_TAG_ENGINE_H__

#include "tdoa_tag_storage.h"

#define SPEED_OF_LIGHT 299792458.0
#define UWB_TS_FREQ (499.2e6 * 128)

typedef void (*tdoaEngineSendTdoaToEstimator)(tdoaMeasurement_t *tdoaMeasurement);

typedef struct
{
  // State
  tdaoAnchorInfoArray_t anchorInfoArray;
  // tdoaStats_t stats;

  // Configuration
  tdoaEngineSendTdoaToEstimator sendTdoaToEstimator;
  double tsFreq;
} tdoaEngineState_t;

void tdoaEngineInit(tdoaEngineState_t *state, const uint32_t now_ms, tdoaEngineSendTdoaToEstimator sendTdoaToEstimator);

bool tdoaEngineGetAnchorCtxForPacketProcessing(tdoaEngineState_t *engineState, const uint8_t anchorId, const uint32_t currentTime_ms, tdoaAnchorContext_t *anchorCtx);
void tdoaEngineProcessPacket(tdoaEngineState_t *engineState, tdoaAnchorContext_t *anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T);

#endif // __TDOA_TAG_ENGINE_H__
