#ifndef __FANG_H__
#define __FANG_H__

#include "agv_types.h"

/*
 * fang算法：解算三维坐标（x，y，z）
 */
bool fang_3D(tdoaMeasurement_t* tdoa, point_t* pTagCrd);

/*
 * fang算法：利用AGV高度已知的先验条件，解算二维坐标（x，y）
 */
bool fang_2D(tdoaMeasurement_t* tdoa, point_t* pTagCrd);

#endif
