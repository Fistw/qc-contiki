#ifndef __FANG_H__
#define __FANG_H__

#include "agv_types.h"
#include "tdoa_tag_engine.h"

/*
 * 过滤不正常tdoa数据。
 */
bool filterTdoaMeasurement(tdoaMeasurement_t* m);
/*
 * 将收到的tdoaMeasurement数据放入数组，更新标签从相应基站收包数。
 */
int fangPutTdoaMeasurement(tdoaMeasurement_t* measure);
/*
 * 从队列中取出三个TDOA数据，一个主基站，三个从基站。
 */
void fangGetTdoaMeasurement(int idx);
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
void calcTagInnerCoodinate1();
/*
 * 计算标签在内坐标轴下的坐标。
 */
void calcTagInnerCoodinate2();
/*
 * 坐标轴转换（内坐标轴转换为外坐标轴）。
 */
void changeAxisFromInnerToOuter(point_t* pTagCrd);

#endif
