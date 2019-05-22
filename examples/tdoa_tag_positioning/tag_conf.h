#ifndef TAG_CONF_H_
#define TAG_CONF_H_

#include "project_common_conf.h"

// AGV标签、人共有配置
// AGV搭载UWB标签高度
#define AGV_Z_AXIS_CONFIG       0.3

// 阈值设置
// 使用AGV坐标z值过滤离群点，默认值=1
#define Z_AXIS_FILTER_CONFIG    1
// 检测基站共面，cos=0.4
#define COPLANAR_FILTER_CONFIG  0.4

// 人私有配置
// 发射功率（默认值=33.5db）
#define txPower_CONFIG          0x1F1F1F1F

#endif /* TAG_CONF_H_ */
