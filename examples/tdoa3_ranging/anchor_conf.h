#ifndef ANCHOR_CONF_H_
#define ANCHOR_CONF_H_

#include "project_common_conf.h"

// tx配置（默认信道4，发射功率33.5db）
#define txPGdly_CONFIG              0xC0
#define txPower_CONFIG              0x1F1F1F1F
// 基站ID
#define anchorID_CONFIG             0x0a
// 基站坐标
#define ANCHOR_AXIS_CONFIG    		{0, 0, 0.551}

#endif /* ANCHOR_CONF_H_ */
