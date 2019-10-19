#include "ls.h"
#include <math.h>
#include <stdio.h>
#include "tag_conf.h"
#include "arm_math.h"
#include "tdoa_tag_storage.h"

static bool filterTdoaMeasurement(tdoaMeasurement_t* m);
static bool lsGetTdoaMeasurement(tdoaMeasurement_t* pTdoaTriad, int count, tdoaMeasurement_t* tdoa);
static bool calcTagCoodinate(tdoaMeasurement_t* pTdoaTriad, const int count, point_t* pTagCrd);

static void createTdoaMeasurement(tdoaMeasurement_t* src, tdoaMeasurement_t* dst);

static float multipleVector(point_t* v1, point_t* v2);
static void createVector(point_t* pstart, point_t* pend, point_t* vector);

/*
 * 过滤不正常tdoa数据。
 */
static bool filterTdoaMeasurement(tdoaMeasurement_t* m)
{
    point_t v;
    createVector(&m->anchorPosition[0], &m->anchorPosition[1], &v);

	return (sqrtf(multipleVector(&v, &v)) > fabsf(m->distanceDiff));
}

/*
 * ls算法
 */
bool ls(int count, tdoaMeasurement_t* tdoa, point_t* pTagCrd){
	tdoaMeasurement_t tdoaTriad[count];
	if(lsGetTdoaMeasurement(tdoaTriad, count, tdoa)){
	    return calcTagCoodinate(tdoaTriad, count, pTagCrd);
	}
	return false;
}

/*
 * 复制tdoa元组。
 */
static bool lsGetTdoaMeasurement(tdoaMeasurement_t* pTdoaTriad, int count, tdoaMeasurement_t* tdoa)
{
    int i;
    for(i = 0; i < count; i++){
        if(filterTdoaMeasurement(&tdoa[i]))
            createTdoaMeasurement(&tdoa[i], &pTdoaTriad[i]);
        else{
            printf("filterTdoaMeasurement: tdoa[%d] is filtered.\n", i);
            return false;
        }
    }
    return true;
//    //检测共面
//    point_t *a1, *a2, *a3, *a4, v12, v13, v14;
//
//    a1 = &tdoaTriad[0].anchorPosition[0];
//    a2 = &tdoaTriad[0].anchorPosition[1];
//    a3 = &tdoaTriad[1].anchorPosition[1];
//    a4 = &tdoaTriad[2].anchorPosition[1];
//
//    createVector(a1, a2, &v12);
//    createVector(a1, a3, &v13);
//    createVector(a1, a4, &v14);
//
//    point_t vn = {.x=v12.y*v13.z-v13.y*v12.z,
//                  .y=v13.x*v12.z-v12.x*v13.z,
//                  .z=v12.x*v13.y-v13.x*v12.y};
//    //arccos0.4
//    return(fabs(multipleVector(&vn, &v14)/(sqrtf(multipleVector(&vn, &vn))*sqrtf(multipleVector(&v14, &v14))))
//            > COPLANAR_FILTER_CONFIG ? true : false);
}

/*
 * 计算标签二维坐标。
 */
static bool calcTagCoodinate(tdoaMeasurement_t* pTdoaTriad, const int count, point_t* pTagCrd)
{
    float argsA[count-1][2], argsb[count-1];
    arm_matrix_instance_f32 A = {.numRows = count-1,
								 .numCols = 2,
								 .pData = argsA};
	arm_matrix_instance_f32 b = {.numRows = count-1,
								 .numCols = 1,
								 .pData = argsb};

	float R21 = pTdoaTriad[0].distanceDiff;

	float k1,k2;
	k1 = multipleVector(&pTdoaTriad[0].anchorPosition[0],&pTdoaTriad[0].anchorPosition[0]);
	k2 = multipleVector(&pTdoaTriad[0].anchorPosition[1],&pTdoaTriad[0].anchorPosition[1]);

    float x1,y1,z1,x2,y2,z2;
    x1 = pTdoaTriad[0].anchorPosition[0].x,y1 = pTdoaTriad[0].anchorPosition[0].y,z1 = pTdoaTriad[0].anchorPosition[0].z;
    x2 = pTdoaTriad[0].anchorPosition[1].x,y2 = pTdoaTriad[0].anchorPosition[1].y,z2 = pTdoaTriad[0].anchorPosition[1].z;

    for(int i = 1; i < count; i++){
    	float Ri1 = pTdoaTriad[i].distanceDiff;

    	float ki = multipleVector(&pTdoaTriad[i].anchorPosition[1],&pTdoaTriad[i].anchorPosition[1]);

    	float xi,yi,zi;
    	xi = pTdoaTriad[i].anchorPosition[1].x,yi = pTdoaTriad[i].anchorPosition[1].y,zi = pTdoaTriad[i].anchorPosition[1].z;

    	argsA[i-1][0] = 2*R21*(xi - x1) - 2*Ri1*(x2 - x1);
    	argsA[i-1][1] = 2*R21*(yi - y1) - 2*Ri1*(y2 - y1);
    	argsb[i-1] = powf(R21, 2)*Ri1 - R21*powf(Ri1, 2) + R21*(ki - k1) + Ri1*(k1 - k2) + AGV_Z_AXIS_CONFIG*(2*Ri1*(z2 - z1) - 2*R21*(zi - z1));
    }
    float tmp1a[2][count-1];
	arm_matrix_instance_f32 tmp1m = {.numRows = 2,
									 .numCols = count-1,
									 .pData = tmp1a};
	float tmp2a[2][2];
	arm_matrix_instance_f32 tmp2m = {.numRows = 2,
									 .numCols = 2,
									 .pData = tmp2a};
	float tmp3a[2][2];
	arm_matrix_instance_f32 tmp3m = {.numRows = 2,
									 .numCols = 2,
									 .pData = tmp3a};
	float tmp4a[2][count-1];
	arm_matrix_instance_f32 tmp4m = {.numRows = 2,
									 .numCols = count-1,
									 .pData = tmp4a};
	float tmp5a[2];
	arm_matrix_instance_f32 tmp5m = {.numRows = 2,
									 .numCols = 1,
									 .pData = tmp5a};
	arm_status flag;
	if((flag = arm_mat_trans_f32(&A, &tmp1m)) != ARM_MATH_SUCCESS){
		printf("Execute trans(A) error, %d\n", flag);
		return false;
	}
	if((flag = arm_mat_mult_f32(&tmp1m, &A, &tmp2m)) != ARM_MATH_SUCCESS){
		printf("Execute mult(A', A) error, %d\n", flag);
		return false;
	}
	if((flag = arm_mat_inverse_f32(&tmp2m, &tmp3m)) != ARM_MATH_SUCCESS){
		printf("Execute inverse(A'*A) error, %d\n", flag);
		return false;
	}
	if((flag = arm_mat_mult_f32(&tmp3m, &tmp1m, &tmp4m)) != ARM_MATH_SUCCESS){
		printf("Execute mult((A'*A)-1, A') error, %d\n", flag);
		return false;
	}
	if((flag = arm_mat_mult_f32(&tmp4m, &b, &tmp5m)) != ARM_MATH_SUCCESS){
		printf("Execute mult((A'*A)-1*A', b) error, %d\n", flag);
		return false;
	}
	pTagCrd->x = tmp5a[0];
	pTagCrd->y = tmp5a[1];
	return true;
}

static void createTdoaMeasurement(tdoaMeasurement_t* src, tdoaMeasurement_t* dst)
{
    setAnchorPosition(&src->anchorPosition[0], &dst->anchorPosition[0]);
    setAnchorPosition(&src->anchorPosition[1], &dst->anchorPosition[1]);
    dst->distanceDiff = src->distanceDiff;
//    dst->distance = src->distance;
    dst->idA = src->idA;
    dst->idB = src->idB;
//    dst->endOfLife = src->endOfLife;
}

static void createVector(point_t* pstart, point_t* pend, point_t* vector)
{
    vector->x = pend->x - pstart->x;
    vector->y = pend->y - pstart->y;
    vector->z = pend->z - pstart->z;
}

static float multipleVector(point_t* v1, point_t* v2)
{
    return (v1->x)*(v2->x)+(v1->y)*(v2->y)+(v1->z)*(v2->z);
}
