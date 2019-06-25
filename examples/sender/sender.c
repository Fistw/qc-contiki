#include "contiki.h"
#include "dw1000.h"
#include "shell.h"
#include "serial-shell.h"
#include <stdio.h>
#include "tdoa/tdoa_decadriver.h"
#include "tdoa/uwb_tdoa_anchor3.h"
#include "tdoa/uwb.h"

#include "dev/leds.h"

dwt_config_t radio_config = {
    .chan = 2,
    .prf = DWT_PRF_64M,
    .txPreambLength = DWT_PLEN_128,
    .dataRate = DWT_BR_6M8,
    .txCode = 9,
    .rxCode = 9,
    .rxPAC = DWT_PAC8,
    .nsSFD = 0 /* standard */,
    .phrMode = DWT_PHRMODE_STD,
    .sfdTO = (129 + 8 - 8),
};

dwt_txconfig_t txcfg = {
		0xC2,
		0x80808080,
};

PROCESS(toda3_ranging, "Tdoa3 ranging");
AUTOSTART_PROCESSES(&toda3_ranging);

PROCESS_THREAD(toda3_ranging, ev, data)
{
    static uwbConfig_t *uwbConfig;
    static dwDevice_t *dev;
    static uint32_t timeout_ms;
//    static uint32_t timeout = 30;
    static struct etimer et;
    PROCESS_BEGIN();
    serial_shell_init();
    dw1000_configure(&radio_config, &txcfg);
    uwbConfig = uwbGetConfig();
    tdoa3Init(uwbConfig);
//    timeout_ms = 1000;
    while (1)
    {
        etimer_set(&et, timeout_ms);
        PROCESS_YIELD_UNTIL(etimer_expired(&et));
//        printf("start: %d\n", clock_time());
//        printf("timeout_ms: %u\n", timeout_ms);
//        timeout_ms = tdoa3UwbEvent(dev);
        timeout_ms = tdoa3UwbEvent(dev);
    }
    PROCESS_END();
}
