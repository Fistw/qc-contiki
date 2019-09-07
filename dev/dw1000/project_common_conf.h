#ifndef PROJECT_COMMON_CONF_H_
#define PROJECT_COMMON_CONF_H_

// 编译时，保留相应类型预定义，注释其余两个预定义
// UWB AGV标签
#define UWB_TYPE_AGV_CONFIG
// UWB人员标签
//#define UWB_TYPE_PERSON_CONFIG
// UWB基站
//#define UWB_TYPE_ANCHOR_CONFIG

/*
 * 共有配置
 */
// 射频基本设置（默认4信道，110K）
#define chan_CONFIG             1
#define prf_CONFIG              DWT_PRF_64M
#define txPreambLength_CONFIG   DWT_PLEN_2048
#define dataRate_CONFIG         DWT_BR_110K
#define txCode_CONFIG           12
#define rxCode_CONFIG           12
#define rxPAC_CONFIG            DWT_PAC64
#define nsSFD_CONFIG            0  
#define phrMode_CONFIG          DWT_PHRMODE_STD
// sfdTO=preamble length + 1 + SFD length - PAC size 
#define sfdTO_CONFIG            (2049 + 64 - 64)

// 发射功率（默认值=33.5db）
#define txPGdly_CONFIG          0xC9
#define txPower_CONFIG          0x1F1F1F1F

//天线延迟
#define antennadelay_CONFIG     16485

/*
 * AGV标签独有配置
 */
// AGV搭载UWB标签高度
#define AGV_Z_AXIS_CONFIG       0.2
// 阈值设置
// 使用AGV坐标z值过滤离群点，默认值=1
#define Z_AXIS_FILTER_CONFIG    5
// 检测基站共面，cos=0.4
#define COPLANAR_FILTER_CONFIG  0.4
// CIR检测阈值
#define NTM_CONFIG              0x69

/*
 * 基站独有配置
 */
// 基站ID
#define anchorID_CONFIG             0x03
// 基站坐标
#define ANCHOR_AXIS_CONFIG    		{0, 5.034, 0}
// 发包频率
#define TX_FREQUENCY_CONFIG			100

#endif /* PROJECT_COMMON_CONF_H_ */
