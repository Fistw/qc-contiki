#ifndef PROJECT_COMMON_CONF_H_
#define PROJECT_COMMON_CONF_H_

// 射频基本设置
#define chan_CONFIG             5
#define prf_CONFIG              DWT_PRF_16M
#define txPreambLength_CONFIG   DWT_PLEN_128
#define dataRate_CONFIG         DWT_BR_6M8
#define txCode_CONFIG           7
#define rxCode_CONFIG           7
#define rxPAC_CONFIG            DWT_PAC8
#define nsSFD_CONFIG            0  
#define phrMode_CONFIG          DWT_PHRMODE_STD
#define sfdTO_CONFIG            (129 + 8 - 8)

// UWB信标类型（0x01:基站，0x02:AGV标签，0x04:人）
#define UWB_TYPE_CONFIG         0x02

#endif /* PROJECT_COMMON_CONF_H_ */
