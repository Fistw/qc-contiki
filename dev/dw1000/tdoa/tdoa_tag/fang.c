#include "fang.h"
#include <math.h>
#include <stdio.h>

static tdoaMeasurement_t tdoaTriad[3];
static double distances[3][3];
static point_t innerAxisQuad[5];//四个基站加一个标签的内坐标轴坐标。

static bool filterTdoaMeasurement(tdoaMeasurement_t* m);

static void createTdoaMeasurement(tdoaMeasurement_t* src, tdoaMeasurement_t* dst);
static void inverseTdoaMeasurement(tdoaMeasurement_t* t);

static bool judgeZAxis();
static double getMinXAxis();
static double getMaxXAxis();

static double multipleVector(point_t* v1, point_t* v2);
static void createVector(point_t* pstart, point_t* pend, point_t* vector);

/*
 * 过滤不正常tdoa数据。
 */
static bool filterTdoaMeasurement(tdoaMeasurement_t* m)
{
    point_t v;
    createVector(&m->anchorPosition[0], &m->anchorPosition[1], &v);

	return (sqrt(multipleVector(&v, &v)) > fabsf(m->distanceDiff));
}

/*
 * fang算法
 */
bool fang(tdoaMeasurement_t* tdoa, point_t* pTagCrd){
	if(fangGetTdoaMeasurement(tdoa)){
		getAnchorDistances();
		createInnerAxis();
		return calcTagInnerCoodinate(pTagCrd);
	}
	return false;
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

    //检测共面
    point_t *a1, *a2, *a3, *a4, v12, v13, v14;

    a1 = &tdoaTriad[0].anchorPosition[0];
    a2 = &tdoaTriad[0].anchorPosition[1];
    a3 = &tdoaTriad[1].anchorPosition[1];
    a4 = &tdoaTriad[2].anchorPosition[1];

    createVector(a1, a2, &v12);
    createVector(a1, a3, &v13);
    createVector(a1, a4, &v14);

    point_t vn = {.x=v12.y*v13.z-v13.y*v12.z,
                  .y=v13.x*v12.z-v12.x*v13.z,
                  .z=v12.x*v13.y-v13.x*v12.y};
    //arccos0.4
    return(fabs(multipleVector(&vn, &v14)/(sqrt(multipleVector(&vn, &vn))*sqrt(multipleVector(&v14, &v14))))>0.4 ? true : false);
}

/*
 * 获取基站测距。
 */
void getAnchorDistances(void)
{
     uint8_t id1, id2, id3, id4;
    point_t *a1, *a2, *a3, *a4, v12, v13, v14, v23, v24, v34;
    double D12, D13, D14, D23, D24, D34;
    
    a1 = &tdoaTriad[0].anchorPosition[0];
    a2 = &tdoaTriad[0].anchorPosition[1];
    a3 = &tdoaTriad[1].anchorPosition[1];
    a4 = &tdoaTriad[2].anchorPosition[1];

    createVector(a1, a2, &v12);
    createVector(a1, a3, &v13);
    createVector(a1, a4, &v14);
    createVector(a2, a3, &v23);
    createVector(a2, a4, &v24);
    createVector(a3, a4, &v34);

    D12 = sqrt(multipleVector(&v12, &v12));
    D13 = sqrt(multipleVector(&v13, &v13));
    D14 = sqrt(multipleVector(&v14, &v14));
    D23 = sqrt(multipleVector(&v23, &v23));
    D24 = sqrt(multipleVector(&v24, &v24));
    D34 = sqrt(multipleVector(&v34, &v34));

    distances[0][0] = D12, distances[0][1] = D13, distances[0][2] = D14;
    distances[1][1] = D23,distances[1][2] = D24;
    distances[2][2] = D34;

    id1 = tdoaTriad[0].idA, id2 = tdoaTriad[0].idB, id3 = tdoaTriad[1].idB, id4 = tdoaTriad[2].idB;
}

/*
 * 建立内坐标轴。
 */
void createInnerAxis()
{
    double *x2 = &innerAxisQuad[1].x;
    double *x3 = &innerAxisQuad[2].x, *y3 = &innerAxisQuad[2].y;
    double *x4 = &innerAxisQuad[3].x, *y4 = &innerAxisQuad[3].y, *z4 = &innerAxisQuad[3].z;
    
    double D12, D13, D14, D23, D24, D34;
    D12 = distances[0][0],D13 = distances[0][1],D14 = distances[0][2];
    D23 = distances[1][1],D24 = distances[1][2];
    D34 = distances[2][2];

    const double pD12 = pow(D12,2), pD13 = pow(D13,2), pD14 = pow(D14,2);
    //运算优化
    *x2 = D12;
//    *x3 = (pow(D23,2)-pD13-pD12) / (-2*D12);
    *x3 = (pow(D23,2)-pD13) / (-2*D12) + D12/2;
    *y3 = sqrt(pD13 - pow(*x3, 2));
//    *x4 = (pow(D24,2)-pD14-pD12) / (-2*D12);
    *x4 = (pow(D24,2)-pD14) / (-2*D12) + D12/2;
//    *y4 = ((-2*(*x3)+2*D12)*(*x4)+pow(*x3,2)+pow(*y3,2)-pD12-pow(D34,2)+pow(D24,2)) / (2*(*y3));
    *y4 = ((-2*(*x3)+2*D12)*(*x4)+pow(*x3,2)-pD12-pow(D34,2)+pow(D24,2)) / (2*(*y3)) + *y3/2;
    *z4 = sqrt(pD14 - pow(*x4,2) - pow(*y4,2));
    
    //判断从基站4的z轴坐标正负性。
    *z4 = judgeZAxis() ? *z4 : -(*z4);
}

/*
 * 计算标签在内坐标轴下的坐标。
 */
bool calcTagInnerCoodinate(point_t* pTagCrd)
{
    double R21, R31, R41;
    R21 = tdoaTriad[0].distanceDiff;
    R31 = tdoaTriad[1].distanceDiff;
    R41 = tdoaTriad[2].distanceDiff;
    
    const double x2 = innerAxisQuad[1].x;
    const double x3 = innerAxisQuad[2].x, y3 = innerAxisQuad[2].y;
    const double x4 = innerAxisQuad[3].x, y4 = innerAxisQuad[3].y, z4 = innerAxisQuad[3].z;

    const double px2 = pow(x2, 2), px3 = pow(x3, 2), py3 = pow(y3, 2);
    const double pR21 = pow(R21, 2), pR31 = pow(R31, 2);

    double g, h, k, l;
    //运算优化
	g = (R31*x2)/(R21*y3) - x3/y3;
	h = ((px3+py3+R21*R31)-(pR31+R31*px2/R21))/(2*y3);
	k = ((R41*x2*y3+R21*x3*y4)-(R21*x4*y3+R31*x2*y4))/(R21*y3*z4);
	l = ((pow(x4,2)+pow(y4,2)+pow(z4,2)+R21*R41)/(2*z4)+(pR31*y4)/(2*y3*z4)+(R31*px2*y4)/(2*R21*y3*z4))
		- ((pow(R41,2)/(2*z4))+(y4*(px3+py3))/(2*y3*z4)+(R21*R31*y4)/(2*y3*z4)+(R41*px2*y3)/(2*R21*y3*z4));
//	printf("g=%f, h=%f, k=%f, l=%f\n",g,h,k,l);

	double d, e, f;

	d = 4*pR21*(1+pow(g,2)+pow(k,2))-4*px2;
	e = (8*pR21*(g*h+l*k)+4*pow(x2,3))-4*pR21*x2;
	f = 4*pR21*(pow(h,2)+pow(l,2)+px2/2)-(pow(pR21,2)+pow(px2,2));
//    printf("d = %f, e = %f, f = %f\n", d, e, f);

	double *x, *y, *z, tmp1,tmp2,drt;
    x = &innerAxisQuad[4].x, y = &innerAxisQuad[4].y, z = &innerAxisQuad[4].z;
	drt = pow(e,2)-4*d*f;
	if(isnormal(drt) && (fabs(drt-0) < 1e-6 || drt > 0)){
		if(fabs(drt-0) < 1e-6)
			printf("tmp3 == 0\t");
		tmp1 = (-e+sqrt(drt))/(2*d);
		tmp2 = (-e-sqrt(drt))/(2*d);

		//点的取舍
		point_t tmpCrd1, tmpCrd2;
		//第一个点
		*x = tmp1;
		*y = g*(*x)+h;
		*z = k*(*x)+l;
		changeAxisFromInnerToOuter(&tmpCrd1);
		//第二个点
		*x = tmp2;
		*y = g*(*x)+h;
		*z = k*(*x)+l;
		changeAxisFromInnerToOuter(&tmpCrd2);

		//z轴过滤阈值=1
		if(fabs(tmpCrd1.z - STD_Z_AXIS) <= 1 || fabs(tmpCrd2.z - STD_Z_AXIS) <= 1){
			if(fabs(tmpCrd1.z - STD_Z_AXIS) < fabs(tmpCrd2.z - STD_Z_AXIS)){
				pTagCrd->x = tmpCrd1.x;
				pTagCrd->y = tmpCrd1.y;
				pTagCrd->z = tmpCrd1.z;
			}else{
				pTagCrd->x = tmpCrd2.x;
				pTagCrd->y = tmpCrd2.y;
				pTagCrd->z = tmpCrd2.z;
			}
			return true;
		}
	}
	return false;
}

/*
 * 坐标轴转换（内坐标轴转换为外坐标轴）。
 */
void changeAxisFromInnerToOuter(point_t* pTagCrd)
{
    double a, b, c;
    double x2i, x3i, y3i, x4i, y4i, z4i, xi, yi, zi;
    x2i = innerAxisQuad[1].x;
    x3i = innerAxisQuad[2].x, y3i = innerAxisQuad[2].y;
    x4i = innerAxisQuad[3].x, y4i = innerAxisQuad[3].y, z4i = innerAxisQuad[3].z;
    xi = innerAxisQuad[4].x, yi = innerAxisQuad[4].y, zi = innerAxisQuad[4].z;
//    printf("x2i=%f,x3i=%f,y3i=%f,x4i=%f,y4i=%f,z4i=%f,xi=%f,yi=%f,zi=%f\n",x2i,x3i,y3i,x4i,y4i,z4i,xi,yi,zi);
    
    c = zi/z4i;
    b = (yi-y4i*c)/y3i;
    a = (xi-x3i*b-x4i*c)/x2i;

    point_t v12, v13, v14;
    createVector(&tdoaTriad[0].anchorPosition[0], &tdoaTriad[0].anchorPosition[1], &v12);
    createVector(&tdoaTriad[0].anchorPosition[0], &tdoaTriad[1].anchorPosition[1], &v13);
    createVector(&tdoaTriad[0].anchorPosition[0], &tdoaTriad[2].anchorPosition[1], &v14);
//    printf("v12(%f,%f,%f),v13(%f,%f,%f),v14(%f,%f,%f)\n",v12.x,v12.y,v12.z,v13.x,v13.y,v13.z,v14.x,v14.y,v14.z);
    
    double array[3] = {0};
    pTagCrd->x = v12.x*a+v13.x*b+v14.x*c+tdoaTriad[0].anchorPosition[0].x;
    pTagCrd->y = v12.y*a+v13.y*b+v14.y*c+tdoaTriad[0].anchorPosition[0].y;
    pTagCrd->z = v12.z*a+v13.z*b+v14.z*c+tdoaTriad[0].anchorPosition[0].z;
    pTagCrd->timestamp = clock_time();
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

static bool judgeZAxis()
{
    point_t v12, v13, v14, n;
    createVector(&tdoaTriad[0].anchorPosition[0], &tdoaTriad[0].anchorPosition[1], &v12);
    createVector(&tdoaTriad[0].anchorPosition[0], &tdoaTriad[1].anchorPosition[1], &v13);
    createVector(&tdoaTriad[0].anchorPosition[0], &tdoaTriad[2].anchorPosition[1], &v14);
//    printf("v12(%f,%f,%f),v13(%f,%f,%f),v14(%f,%f,%f)\n",v12.x,v12.y,v12.z,v13.x,v13.y,v13.z,v14.x,v14.y,v14.z);
    n.x = v12.y*v13.z-v13.y*v12.z;
    n.y = v13.x*v12.z-v12.x*v13.z;
    n.z = v12.x*v13.y-v13.x*v12.y;

//    double cosVal;
//    cosVal = multipleVector(&n, &v14) / sqrt(multipleVector(&n, &n)*multipleVector(&v14, &v14));
//
//    return (cosVal > 0) ? true : false;
    return (multipleVector(&n, &v14) > 0) ? true : false;
}

static double getMinXAxis()
{
    int i;
    double res = 0;
    for(i = 1; i < 4; i++){
        if(innerAxisQuad[i].x < res)
            res = innerAxisQuad[i].x;
    }
    return res;
}

static double getMaxXAxis()
{
    int i;
    double res = 0;
    for(i = 1; i < 4; i++){
        if(innerAxisQuad[i].x > res)
            res = innerAxisQuad[i].x;
    }
    return res;
}

static void createVector(point_t* pstart, point_t* pend, point_t* vector)
{
    vector->x = pend->x - pstart->x;
    vector->y = pend->y - pstart->y;
    vector->z = pend->z - pstart->z;
}

static double multipleVector(point_t* v1, point_t* v2)
{
    return (v1->x)*(v2->x)+(v1->y)*(v2->y)+(v1->z)*(v2->z);
}
