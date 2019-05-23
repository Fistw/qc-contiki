#include "uwb.h"
#include "tdoa_decadriver.h"
#include "anchor_conf.h"

// System configuration
static struct uwbConfig_s config =
    {
        address :
            {anchorID_CONFIG, 0, 0, 0, 0, 0, 0xcf, 0xbc},
    };

struct uwbConfig_s *uwbGetConfig()
{
    config.positionEnabled = 1;
    return &config;
}

static dwDevice_t device = {

};

dwDevice_t *uwbGetDevice()
{
    return &device;
}
