#include "contiki.h"
#include "dw1000.h"
#include <stdio.h>
#include "tdoa/tdoa_decadriver.h"
#include "tdoa/uwb_tdoa_anchor3.h"
#include "tdoa/uwb.h"


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

PROCESS(toda3_ranging, "Tdoa3 ranging");
AUTOSTART_PROCESSES(&toda3_ranging);

PROCESS_THREAD(toda3_ranging, ev, data)
{
    static uwbConfig_t *uwbConfig;
    // 设置发射功率：15db=0010,0000、33.5db=0001,1111
    static dwt_txconfig_t txConfig = {.PGdly=0xC0, .power=0x20202020};
    static dwDevice_t *dev;
    static uint32_t timeout_ms;
    // static uint32_t timeout = 30;
    static struct etimer et;
    PROCESS_BEGIN();
    dw1000_configure(&radio_config);
    /* 关闭智能功率调节 */
    dwt_setsmarttxpower(0);
    dwt_configuretxrf(&txConfig);
    printf("Process begin\n");
    uwbConfig = uwbGetConfig();
    tdoa3Init(uwbConfig);
    timeout_ms = tdoa3UwbEvent(dev);
    while (1)
    {
        etimer_set(&et, timeout_ms);
        printf("\n\nBefore etimer_expired ::: %u\n", clock_time());
        PROCESS_YIELD_UNTIL(etimer_expired(&et));
        printf("After etimer_expired ::: %u\n", clock_time());
        // printf("start: %d\n", clock_time());
        printf("timeout_ms ::: %u\n", timeout_ms);
        timeout_ms = tdoa3UwbEvent(dev);
        // printf("end: %d\n", clock_time());
    }
    PROCESS_END();
}
