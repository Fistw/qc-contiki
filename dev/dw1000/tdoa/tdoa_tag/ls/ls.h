#ifndef __LS_H__
#define __LS_H__

#include "agv_types.h"

/*
 * LS算法：解算二维坐标（x，y）
 */
bool ls(int count, tdoaMeasurement_t* tdoa, point_t* pTagCrd);

#endif
