#include "fang.h"
#include <math.h>

tdoaQueue_t queue;

float A[3][3];
arm_matrix_instance_f32 Am = {3, 3, (float*)A};

float b[3];

static uint8_t ids[TDOA_ANCHOR_COUNT];
static uint8_t packets[TDOA_ANCHOR_COUNT];

uint8_t getAnchorIdIndex(uint8_t id)
{
    int i;
    for(i = 0; i < TDOA_ANCHOR_COUNT; i++){
        if (ids[i] == id)
            return i;
    }
    return -1;
}

void setAnchorPosition(point_t* src, point_t* dst)
{
    dst.x = src.x;
    dst.y = src.y;
    dst.z = src.z;
}

void setTdoaMeasurement(tdoaMeasurement_t* src, tdoaMeasurement_t* dst)
{
    setAnchorPosition(&src.anchorPosition[0], &dst.anchorPosition[0]);
    setAnchorPosition(&src.anchorPosition[1], &dst.anchorPosition[1]);
    dst.distanceDiff = src.distanceDiff;
    dst.idA = src.idA;
    dst.idB = src.idB;
    dst.endOfLife = clock_time();
}

/*
 * 将收到的tdoaMeasurement数据放入数组，更新标签从相应基站收包数。
 */
int fangPutTdoaMeasurementToQueue(tdoaMeasurement_t* pQueue, tdoaMeasurement_t* measure)
{
    int sameSlot = -1, firstEmptySlot = -1, expiredSlot = 0;
    uint8_t idA = measure->idA, idB = measure->idB;
    int idxA, idxB, res;
    uint32_t expiredTime = clock_time();
    int i;

    for(i = 0; i < TDOA_QUEUE_LENTH; i++){
        if(!pQueue[i].used){
            if(firstEmptySlot != -1)
                firstEmptySlot = i;
        }else if(pQueue[i].idA == idA && pQueue[i].idB == idB){
            sameSlot = i;
        }else if(pQueue[i].endOfLife < expiredTime){
            pQueue[i].used = 0x0;
            int idxC, idxD;
            if((idxC = getAnchorIdIndex(pQueue[i].idA)) != -1)
                packets[idxC]--;
            if((idxD = getAnchorIdIndex(pQueue[i].idB)) != -1)
                packets[idxD]--;
            expiredSlot = i;
        }
    }

    if(sameSlot != -1){
        setTdoaMeasurement(measure, &pQueue[sameSlot]);
    }else if(firstEmptySlot != -1){
        setTdoaMeasurement(measure, &pQueue[firstEmptySlot]);
        pQueue[firstEmptySlot].used = 0x1;
    }else{
        setTdoaMeasurement(measure, &pQueue[expiredSlot]);
        pQueue[expiredSlot].used = 0x1;
    }

    if((idxA = getAnchorIdIndex(idA)) != -1)
        packets[idxA]++;
    if((idxB = getAnchorIdIndex(idB)) != -1)
        packets[idxB]++;
    return (packets[idxA] >= 0x6) ? idxA : ((packets[idxB] >= 0x6) ? idxB : -1);
}

void inverseTdoaMeasurement(tdoaMeasurement_t* t)
{
    point_t tmpPoint;
    uint8_t tmpId;

    setAnchorPosition(&t.anchorPosition[0], &tmpPoint);
    setAnchorPosition(&t.anchorPosition[1], &t.anchorPosition[0]);
    setAnchorPosition(&tmpPoint, &t.anchorPosition[1]);
    
    tmpId = t.idA;
    t.idA = t.idB;
    t.idB = tmpId;

    t.distanceDiff = -t.distanceDiff;
}

void fangPutOnelineMatrix(tdoaMeasurement_t* measureA, tdoaMeasurement_t* measureB, float* A, float* b, int flag)
{
    float x, y, z, xi, yi, zi, xj, yj, zj;
    x = measureA->position[0].x, y = measureA->position[0].y, z = measureA->position[0].z;
    xi = measureA->position[1].x, yi = measureA->position[1].y, zi = measureA->position[1].z;
    xj = measureB->position[1].x, yj = measureB->position[1].y, zj = measureB->position[1].z;

    float Di, Dj, Xi, Yi, Zi, Xj, Yj, Zj, K, Ki, Kj;
    Di = measureA->distanceDiff, Dj = measureB->distanceDiff;
    Xi = xi-x, Yi = yi-y, Zi = zi-z;
    Xj = xj-x, Yj = yj-y, Zj = zj-z;
    K = powf(x, 2)*powf(y, 2)*powf(z, 2), Ki = powf(xi, 2)*powf(yi, 2)*powf(zi, 2), Kj = powf(xj, 2)*powf(yj, 2)*powf(zj, 2);
    
    *(A+flag*3) = -2*Dj*Xi+2*Di*Xj, *(A+flag*3+1) = -2*Dj*Yi+2*Di*Yj, *(A+flag*3+2) = -2*Dj*Zi+2*Di*Zj;
    *(b+flag) = -Dj*Ki + Di*Kj + (Dj - Di)*K + Di*Dj*(Di - Dj);
}

/*
 * 找出一个主基站与六个从基站的tdoaMeasurement数据，放入矩阵A与b。
 */
void fangPutMatrix(tdoaMeasurement_t* pQueue, int idx)
{
    uint8_t id = ids[idx];
    tdoaMeasurement_t *measureA, *measureB;
    int i, flag1 = 0, flag2 = 0;

    for(i = 0; i < TDOA_QUEUE_LENTH; i++){
        if(pQueue[i].idA == id || pQueue[i].idB == id){
            if(pQueue[i].idB == id)
                inverseTdoaMeasurement(&pQueue[i]);
            if(flag1 == 0){
                measureA = &pQueue[i];
            }else if(flag1 == 1){
                measureB = &pQueue[i];
                fangPutOnelineMatrix(measureA, measureB, (float*)A, (float*)b, flag2);
                flag1 = 0;
            }
            flag2++;
        }
    }
}

/*
 * 计算方程组Ax = b，求出标签坐标。
 */
bool calcTagCoordinate(arm_matrix_instance_f32* Am, float* b, point_t* tagCrd)
{
    float tmp[3][3];
    arm_matrix_instance_f32 tmpm = {3, 3, (float*)tmp};
    
    if(arm_mat_inverse_f32(Am, tmpm) == ARM_MATH_SUCCESS){
        int i, j;
        float array[3] = {0};
        for(i = 0; i < 3; i++){
            for(j = 0; j < 3; j++){
                array[i] += tmp[i][j]*b[i];
            }
        }
        tagCrd->x = array[0], tagCrd->y = array[1], tagCrd->z = array[2], tagCrd->timestamp = clock_time();
        return true;
    }
    return false;
}