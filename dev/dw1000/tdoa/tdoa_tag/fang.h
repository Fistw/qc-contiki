#ifndef __FANG_H__
#define __FANG_H__

#include "agv_types.h"
#include "arm_math"

extern tdoaQueue_t queue;

extern float A[3][3];
extern arm_matrix_instance_f32 Am = {3, 3, (float*)A};

float b[3];

int fangPutTdoaMeasurementToQueue(tdoaMeasurement_t* pQueue, tdoaMeasurement_t* measure);
void fangPutMatrix(tdoaMeasurement_t* pQueue, int idx);
bool calcTagCoordinate(arm_matrix_instance_f32* Am, float* b, point_t* tagCrd)

#endif