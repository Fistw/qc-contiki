#include "contiki.h"
#include "dw1000.h"
#include "dw1000/tdoa/tdoa_decadriver.h"

PROCESS(toda3_ranging, "Tdoa3 ranging");
AUTOSTART_PROCESSES(&toda3_ranging);

PROCESS_THREAD(toda3_ranging, ev, data)
{
    static uwbConfig_t *uwbConfig;
    static deDevice_t* dev;
    static uint32_t timeout_ms;
    static struct etimer et;
    PROCESS_BEGIN();
    uwbConfig  = uwbGetConfig();
    tdoa3Init(uwbConfig);
    timeout_ms = tdoa3UwbEvent(dev);
    while(1)
    {
        etimer_set(&et, timeout_ms);
        PROCESS_WAIT_UNTIL(etimer_expired(&et));
        timeout_ms = tdoa3UwbEvent(dev);
    }
    PROCESS_END();
}