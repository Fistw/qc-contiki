#ifndef __UWB_TDOA_ANCHOR3_H__
#define __UWB_TDOA_ANCHOR3_H__

#include "uwb.h"
#include "tdoa_decadriver.h"

void handleRxPacket(const uint8_t *packetbuf, const uint16_t data_len);

uint32_t tdoa3UwbEvent(dwDevice_t *dev);
void tdoa3Init(uwbConfig_t *config);

#endif