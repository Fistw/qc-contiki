#include <stdio.h>

#include "contiki.h"

#include "dw1000.h"
#include "tdoa/tdoa_tag/tdoa_tag.h"

// dwt_config_t为dw1000 deca中定义的
dwt_config_t radio_config = {
    .chan = 5,
    .prf = DWT_PRF_16M,
    .txPreambLength = DWT_PLEN_128,
    .dataRate = DWT_BR_6M8,
    .txCode = 7,
    .rxCode = 7,
    .rxPAC = DWT_PAC8,
    .nsSFD = 0 /* standard */,
    .phrMode = DWT_PHRMODE_STD,
    .sfdTO = (129 + 8 - 8),
};

PROCESS(tdoa_tag_positioning, "Tag position using TDoA");
AUTOSTART_PROCESSES(&tdoa_tag_positioning);

PROCESS_THREAD(tdoa_tag_positioning, ev, data)
{
    PROCESS_BEGIN();

    dw1000_configure(&radio_config);
    printf("Process begin\n");
//    onEvent(2);
    tdoaTagInit();

    PROCESS_END();
}
