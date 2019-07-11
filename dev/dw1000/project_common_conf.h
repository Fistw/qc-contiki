#ifndef PROJECT_COMMON_CONF_H_
#define PROJECT_COMMON_CONF_H_

// 射频基本设置
#define chan_CONFIG             4
#define prf_CONFIG              DWT_PRF_64M
#define txPreambLength_CONFIG   DWT_PLEN_2048
#define dataRate_CONFIG         DWT_BR_110K
#define txCode_CONFIG           17
#define rxCode_CONFIG           17
#define rxPAC_CONFIG            DWT_PAC64
#define nsSFD_CONFIG            0  
#define phrMode_CONFIG          DWT_PHRMODE_STD
// sfdTO=preamble length + 1 + SFD length - PAC size 
#define sfdTO_CONFIG            (2049 + 64 - 64)

//天线延迟
#define antennadelay_CONFIG     16420

// UWB标签类型（当编译UWB基站时，注释此预定义）
//#define UWB_TYPE_TAG_CONFIG

#endif /* PROJECT_COMMON_CONF_H_ */
