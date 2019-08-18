#include "fang.h"
#include <math.h>
#include <stdio.h>
#include "tag_conf.h"
#include "taylor.h"
#include "tdoa_tag_engine.h"

static tdoaMeasurement_t tdoaTriad[3];

static bool filterTdoaMeasurement(tdoaMeasurement_t* m);
static bool fangGetTdoaMeasurement(tdoaMeasurement_t* tdoa);
static bool calcTagCoodinate(point_t* pTagCrd);

static void createTdoaMeasurement(tdoaMeasurement_t* src, tdoaMeasurement_t* dst);
static void inverseTdoaMeasurement(tdoaMeasurement_t* t);

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
 * fang算法
 */
bool fang_2D(tdoaMeasurement_t* tdoa, point_t* pTagCrd){
    bool flag = false;
    if(fangGetTdoaMeasurement(tdoa)){
	    flag = calcTagCoodinate(pTagCrd);
        taylor(tdoaTriad, pTagCrd);
	}
	return flag;
}

/*
 * 复制tdoa三元组。
 */
bool fangGetTdoaMeasurement(tdoaMeasurement_t* tdoa)
{
    int i;
    for(i = 0; i < 3; i++){
        if(filterTdoaMeasurement(&tdoa[i]))
            createTdoaMeasurement(&tdoa[i], &tdoaTriad[i]);
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
bool calcTagCoodinate(point_t* pTagCrd)
{
    float R21, R31, R41;
    R21 = tdoaTriad[0].distanceDiff;
    R31 = tdoaTriad[1].distanceDiff;
    R41 = tdoaTriad[2].distanceDiff;

    float x1,y1,z1,x2,y2,z2,x3,y3,z3,x4,y4,z4;

    x1 = tdoaTriad[0].anchorPosition[0].x,y1 = tdoaTriad[0].anchorPosition[0].y,z1 = tdoaTriad[0].anchorPosition[0].z;
    x2 = tdoaTriad[0].anchorPosition[1].x,y2 = tdoaTriad[0].anchorPosition[1].y,z2 = tdoaTriad[0].anchorPosition[1].z;
    x3 = tdoaTriad[1].anchorPosition[1].x,y3 = tdoaTriad[1].anchorPosition[1].y,z3 = tdoaTriad[1].anchorPosition[1].z;
    x4 = tdoaTriad[2].anchorPosition[1].x,y4 = tdoaTriad[2].anchorPosition[1].y,z4 = tdoaTriad[2].anchorPosition[1].z;

    float a1,b1,c1,a2,b2,c2;
    const float pR21 = powf(R21, 2), pR31 = powf(R31, 2), pR41 = powf(R41, 2);
    float k1,k2,k3,k4;

    k1 = multipleVector(&tdoaTriad[0].anchorPosition[0],&tdoaTriad[0].anchorPosition[0]);
    k2 = multipleVector(&tdoaTriad[0].anchorPosition[1],&tdoaTriad[0].anchorPosition[1]);
    k3 = multipleVector(&tdoaTriad[1].anchorPosition[1],&tdoaTriad[1].anchorPosition[1]);
    k4 = multipleVector(&tdoaTriad[2].anchorPosition[1],&tdoaTriad[2].anchorPosition[1]);

    a1 = 2*R21*(x4-x1)-2*R41*(x2-x1);
    b1 = 2*R21*(y4-y1)-2*R41*(y2-y1);
    c1 = R41*pR21-R21*pR41-R21*(k1-k4)-R41*(k2-k1)-(2*R21*(z4-z1)-2*R41*(z2-z1))*pTagCrd->z;
    a2 = 2*R31*(x4-x1)-2*R41*(x3-x1);
    b2 = 2*R31*(y4-y1)-2*R41*(y3-y1);
    c2 = R41*pR31-R31*pR41-R31*(k1-k4)-R41*(k3-k1)-(2*R31*(z4-z1)-2*R41*(z3-z1))*pTagCrd->z;

    pTagCrd->x = (c2-b2*c1/b1)/(a2-b2*a1/b1);
    pTagCrd->y = (c1-a1*pTagCrd->x)/b1;
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

static void inverseTdoaMeasurement(tdoaMeasurement_t* t)
{
    point_t tmpPoint;
    uint8_t tmpId;

    setAnchorPosition(&t->anchorPosition[0], &tmpPoint);
    setAnchorPosition(&t->anchorPosition[1], &t->anchorPosition[0]);
    setAnchorPosition(&tmpPoint, &t->anchorPosition[1]);
    
    tmpId = t->idA;
    t->idA = t->idB;
    t->idB = tmpId;

    t->distanceDiff = -t->distanceDiff;
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
