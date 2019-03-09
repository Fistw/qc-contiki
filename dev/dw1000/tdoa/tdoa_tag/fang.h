#ifndef __FANG_H__
#define __FANG_H__

#include "agv_types.h"
#include "arm_math"
tdoaQueue_t queue;

float A[3][3];
arm_matrix_instance_f32 Am = {3, 3, (float*)A};

float b[3];

uint8_t flag = 0;

tdoaMeasurement_t* fangGetPutTdoaMeasurement(tdoaMeasurement_t* queue, tdoaMeasurement_t* measure);
void fangPutMatrix(tdoaMeasurement_t* measureA, tdoaMeasurement_t* measureB, float* A, float* b)
bool calcTagCoordinate(arm_matrix_instance_f32* Am, float* b, point_t* tagCrd)

#endif