#ifndef ANCHOR_CONF_H_
#define ANCHOR_CONF_H_

#include "project_common_conf.h"

// tx配置（默认信道4，发射功率33.5db）
#define txPGdly_CONFIG              0x95
#define txPower_CONFIG              0x1F1F1F1F
// 基站ID
#define anchorID_CONFIG             0x0e
// 基站坐标
#define ANCHOR_AXIS_CONFIG    		{0, 1.6, 0}

#endif /* ANCHOR_CONF_H_ */
