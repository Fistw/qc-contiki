#include <stdio.h>

#include "contiki.h"

#include "dw1000.h"
#include "tdoa/tdoa_tag/tdoa_tag.h"
#include "tag_conf.h"

// dwt_config_t为dw1000 deca中定义的
dwt_config_t radio_config = {
    .chan = chan_CONFIG,
    .prf = prf_CONFIG,
    .txPreambLength = txPreambLength_CONFIG,
    .dataRate = dataRate_CONFIG,
    .txCode = txCode_CONFIG,
    .rxCode = rxCode_CONFIG,
    .rxPAC = rxPAC_CONFIG,
    .nsSFD = nsSFD_CONFIG /* standard */,
    .phrMode = phrMode_CONFIG,
    .sfdTO = sfdTO_CONFIG,
};

PROCESS(tdoa_tag_positioning, "Tag position using TDoA");
AUTOSTART_PROCESSES(&tdoa_tag_positioning);

PROCESS_THREAD(tdoa_tag_positioning, ev, data)
{
    PROCESS_BEGIN();

    // 设置发射功率：15db=0010,0000、33.5db=0001,1111
    static dwt_txconfig_t txConfig = {.PGdly=0xC0, .power=txPower_CONFIG};
    /* 关闭智能功率调节 */
    dwt_setsmarttxpower(0);
    dwt_configuretxrf(&txConfig);
    
    dw1000_configure(&radio_config);
    printf("Process begin\n");
//    onEvent(2);
    tdoaTagInit();

    PROCESS_END();
}
