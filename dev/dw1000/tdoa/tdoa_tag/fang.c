#include "fang.h"
#include <math.h>

tdoaMeasurement_t* fangGetPutTdoaMeasurement(tdoaMeasurement_t* queue, tdoaMeasurement_t* measure)
{
    uint32_t oldestTime = clock_time();
    int firstEmptySlot = -1, oldestSlot = 0;
    int i, idA = measure->idA;
    for(i = 0; i < TDOA_QUEUE_LENTH; i++){
        if(!queue[i]){
            if(firstEmptySlot == -1)
                firstEmptySlot = i;
        }else if(queue[i]->endOfLife < oldestTime){
            oldestTime = queue[i]->endOfLife;
            oldestSlot = i;
        }else if(queue[i]->idA == idA){
            return queue[i];
        }
    }
    if(firstEmptySlot != -1)
        queue[firstEmptySlot] = measure;
    else
        queue[oldestSlot] = measure;
    return 0;
}

void fangPutMatrix(tdoaMeasurement_t* measureA, tdoaMeasurement_t* measureB, float* A, float* b)
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

    flag++;
}

bool calcTagCoordinate(arm_matrix_instance_f32* Am, float* b, point_t* tagCrd)
{
    float tmp[3][3];
    arm_matrix_instance_f32 tmpm = {3, 3, (float*)tnmp};
    
    if(arm_mat_inverse_f32(Am, tmpm) == ARM_MATH_SUCCESS){
        int i, j;
        float array[3] = {0};
        for(i = 0; i < 3; i++){
            for(j = 0; j < 3; j++){
                tagCrdArray[i] += tmp[i][j]*b[j];
            }
        }
        tagCrd->x = array[0], tagCrd->y = array[1], tagCrd->z = array[2], tagCrd->timestamp = clock_time();
        return true;
    }
    return false;
}