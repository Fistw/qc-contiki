#ifndef __FANG_H__
#define __FANG_H__

#include "agv_types.h"
#include "tdoa_tag_engine.h"



/*
 * fang算法
 */
bool fang(tdoaMeasurement_t* tdoa, point_t* pTagCrd);
/*
 * 复制tdoa三元组.
 */
bool fangGetTdoaMeasurement(tdoaMeasurement_t* tdoa);
/*
 * 获取基站测距。
 */
void getAnchorDistances(void);
/*
 * 建立内坐标轴。
 */
void createInnerAxis();
/*
 * 计算标签在内坐标轴下的坐标。
 */
bool calcTagInnerCoodinate(point_t* pTagCrd);
/*
 * 坐标轴转换（内坐标轴转换为外坐标轴）。
 */
void changeAxisFromInnerToOuter(point_t* pTagCrd);

#endif
