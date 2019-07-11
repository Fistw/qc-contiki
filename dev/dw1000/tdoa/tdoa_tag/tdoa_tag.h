#ifndef __TDOA_TAG_H__
#define __TDOA_TAG_H__

#include <stdint.h>

void handleTagRxPacket(uint64_t rxTime, const uint8_t *packetbuf, const uint16_t data_len);
void tdoaTagInit();
#endif
