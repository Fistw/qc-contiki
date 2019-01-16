#ifndef __UWB_H__
#define __UWB_H__

#include <stdint.h>
#include <stdbool.h>

typedef struct uwbConfig_s
{
    uint8_t mode;
    uint8_t address[8];
    uint8_t anchorListSize;
    // uint8_t anchors[MAX_ANCHORS];
    uint8_t anchors[8];
    float position[3];
    bool positionEnabled;

    bool smartPower;
    bool forceTxPower;
    uint32_t txPower;

    bool lowBitrate;
    bool longPreamble;
} uwbConfig_t;

struct uwbConfig_s *uwbGetConfig();

#endif