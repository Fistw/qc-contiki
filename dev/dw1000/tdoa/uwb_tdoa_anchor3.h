#ifndef __UWB_TDOA_ANCHOR3_H__
#define __UWB_TDOA_ANCHOR3_H__

#include "uwb.h"
#include "tdoa_decadriver.h"
#include "tag_conf.h"

void handleRxPacket(uint64_t rxTime, const uint8_t *packetbuf, const uint16_t data_len, uint64_t regTxTime);

uint32_t tdoa3UwbEvent(dwDevice_t *dev);
void tdoa3Init(uwbConfig_t *config);
// 便于人员安全避让功能使用
void setupTx(float* array);

// // 声明setupTx，便于人员安全避让功能使用
// void setupTx(dwDevice_t* dev);

#endif
