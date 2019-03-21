#include "fang.h"
#include <math.h>

static tdoaQueue_t queue;

static uint8_t ids[TDOA_ANCHOR_COUNT] = {0};
static int packets[TDOA_ANCHOR_COUNT] = {0};

static tdoaMeasurement_t tdoaTriad[3];
static float distances[3][3];
static point_t innerAxisQuad[5];//四个基站加一个标签的内坐标轴坐标。

static int getAnchorIdIndex(uint8_t id);
static int createAnchorIdIndex(uint8_t id);

static void createTdoaMeasurement(tdoaMeasurement_t* src, tdoaMeasurement_t* dst);
static void inverseTdoaMeasurement(tdoaMeasurement_t* t);

static bool judgeZAxis();
static float getMinXAxis();
static float getMaxXAxis();

static float multipleVector(point_t* v1, point_t* v2);
static void createVector(point_t* pstart, point_t* pend, point_t* vector);

/*
 * 过滤不正常tdoa数据。
 */
bool filterTdoaMeasurement(tdoaMeasurement_t* m)
{
    return (m->distance > fabsf(m->distanceDiff));
}

/*
 * 将收到的tdoaMeasurement数据放入数组，更新标签从相应基站收包数。
 */
int fangPutTdoaMeasurement(tdoaMeasurement_t* measure)
{ 
    int sameSlot = -1, firstEmptySlot = -1, expiredSlot = -1;
    uint8_t idA = measure->idA, idB = measure->idB;
    uint32_t expiredTime = clock_time();
    int idxA, idxB;
    int i;

    // 寻找在queue中可以存放新的tdoa数据的位置
    for(i = 0; i < TDOA_QUEUE_LENTH; i++){
        if(((queue[i].used & 1) == 0)){
            if(firstEmptySlot == -1)
                firstEmptySlot = i;
        }else if(queue[i].endOfLife < expiredTime){
            //同时更新ids数组及packets数组。
            int idxC, idxD;
            if((idxC = getAnchorIdIndex(queue[i].idA)) != -1){
                packets[idxC]--;
            }
            else
                printf("Error: getAnchorIdIndex return -1.\n");

            if((idxD = getAnchorIdIndex(queue[i].idB)) != -1)
                packets[idxD]--;
            else
                printf("Error: getAnchorIdIndex return -1.\n");

            memset(&queue[i], 0, sizeof(tdoaMeasurement_t));
            expiredSlot = i;
        }else if((queue[i].idA == idA) && (queue[i].idB == idB)){
            sameSlot = i;
        }
    }

    if(sameSlot != -1){
        createTdoaMeasurement(measure, &queue[sameSlot]);
        idxA = getAnchorIdIndex(idA);
        idxB = getAnchorIdIndex(idB);
    }else{
        if(firstEmptySlot != -1){
            createTdoaMeasurement(measure, &queue[firstEmptySlot]);
            queue[firstEmptySlot].used |= 1;
        }else{
            createTdoaMeasurement(measure, &queue[expiredSlot]);
            queue[expiredSlot].used |= 1;
        }

        //同时更新ids数组及packets数组。
		if((idxA = getAnchorIdIndex(idA)) == -1){
			if((idxA = createAnchorIdIndex(idA)) == -1)
				printf("Error: createAnchorIdIndex return -1.\n");

		}
		if((idxB = getAnchorIdIndex(idB)) == -1){
			if((idxB = createAnchorIdIndex(idB)) == -1)
				printf("Error: createAnchorIdIndex return -1.\n");
		}

//        printf("****************************************** new tdoa! %d 's packets is %d, ready to plus\n",idA,packets[idxA]);

    	packets[idxA]++;
    	packets[idxB]++;
//    	printf("****************************************** new tdoa! %d 's packets is %d, plus\n",idA,packets[idxA]);
    } 
//    printf("############################sameSlot = %d, firstEmptySlot = %d, oldestSlot = %d\n", sameSlot, firstEmptySlot, expiredSlot);
    for(i = 0; i < 8; i++){
    	if((queue[i].used & 1) == 1){
    		printf("queue: idA is %d, idB is %d\n",queue[i].idA,queue[i].idB);
    	}
    }
//    for(i = 0; i < 8; i++){
//    	printf("ids[%d].id is %d, packets[i].count is %d\n", i, ids[i], packets[i]);
//    }

    return (packets[idxA] >= 3) ? idxA : ((packets[idxB] >= 3) ? idxB : -1);
}

/*
 * 从队列中取出三个TDOA数据，一个主基站，三个从基站。
 */
void fangGetTdoaMeasurement(int idx)
{
    uint8_t id = ids[idx];
    int i, count = 0;
    for(i = 0; i < TDOA_QUEUE_LENTH, count < 3; i++){
        if(((queue[i].used & 1) == 1) && (queue[i].idA == id || queue[i].idB == id)){
            createTdoaMeasurement(&queue[i], &tdoaTriad[count]);
            if(queue[i].idB == id)
                inverseTdoaMeasurement(&tdoaTriad[count]);
//            printf("tdoaTriad:idA is %u, idB is %u\n",tdoaTriad[count].idA,tdoaTriad[count].idB);
//            printf("queue:idA is %u, idB is %u\n",queue[i].idA,queue[i].idB);
            count++;
        }
    }
    printf("Debug: fangGetTdoaMeasurement has executed, count = %d\n", count);
}

/*
 * 获取基站测距。
 */
void getAnchorDistances(void)
{
    // uint8_t id2, id3, id4;
    // tdoaAnchorContext_t t2, t3, t4;
    // uint32_t nowms = clock_time();
    // int64_t tof23, tof24, tof34;
    point_t *a1, *a2, *a3, *a4, v12, v13, v14, v23, v24, v34;
    float D12, D13, D14, D23, D24, D34;
    
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

    D12 = multipleVector(&v12, &v12);
    D13 = multipleVector(&v13, &v13);
    D14 = multipleVector(&v14, &v14);
    D23 = multipleVector(&v23, &v23);
    D34 = multipleVector(&v34, &v34);

    distances[0][0] = D12, distances[0][1] = D13, distances[0][2] = D14;
    distances[1][1] = D23,distances[1][2] = D24;
    distances[2][2] = D34;
    // id2 = tdoaTriad[0].idB, id3 = tdoaTriad[1].idB, id4 = tdoaTriad[2].idB;

    // int i;
	// for(i = 0; i < 3; i++){
	// 	tdoaMeasurement_t* t = &tdoaTriad[i];
	// 	printf("tdoaTriad[%d]: idA=%d,idB=%d,distanceDiff=%f\n",i,t->idA,t->idB,t->distanceDiff);
	// }
	// tdoaTimeOfFlight_t *atof2, *atof3, *atof4;
	// if(tdoaStorageGetAnchorCtx(anchorStorage, id2, nowms, &t2)){
	// 	atof2 = t2.anchorInfo->tof;
	// 	for(i = 0; i < 16; i++){
	// 		if(atof2[i].id != 0)
	// 			printf("tof of id%d and id%d is %lld, endoflife = %u, now = %u\n",id2, (unsigned)atof2[i].id, atof2[i].tof, atof2[i].endOfLife, clock_time());
	// 		}
	// }
	// if(tdoaStorageGetAnchorCtx(anchorStorage, id3, nowms, &t3)){
	// 	atof3 = t3.anchorInfo->tof;
	// 	for(i = 0; i < 16; i++){
	// 		if(atof3[i].id != 0)
	// 			printf("tof of id%d and id%d is %lld, endoflife = %u, now = %u\n",id3, (unsigned)atof3[i].id, atof3[i].tof, atof3[i].endOfLife, clock_time());
	// 		}
	// }
	// if(tdoaStorageGetAnchorCtx(anchorStorage, id4, nowms, &t4)){
	// 	atof4 = t4.anchorInfo->tof;
	// 	for(i = 0; i < 16; i++){
	// 		if(atof4[i].id != 0)
	// 			printf("tof of id%d and id%d is %lld, endoflife = %u, now = %u\n",id4, (unsigned)atof4[i].id, atof4[i].tof, atof4[i].endOfLife, clock_time());
	// 	}
	// }

    // if(tdoaStorageGetAnchorCtx(anchorStorage, id2, nowms, &t2)){
    // 	tof23 = tdoaStorageGetTimeOfFlight(&t2, id3);
    // 	tof24 = tdoaStorageGetTimeOfFlight(&t2, id4);
    // }
    // if(tdoaStorageGetAnchorCtx(anchorStorage, id3, nowms, &t3)){
	// 	if(tof23 == 0)
	// 		tof23 = tdoaStorageGetTimeOfFlight(&t3, id2);
	// 	tof34 = tdoaStorageGetTimeOfFlight(&t3, id4);
    // }
    // if(((tof24 == 0) || (tof34 == 0)) && tdoaStorageGetAnchorCtx(anchorStorage, id4, nowms, &t4)){
	// 	tof24 = tdoaStorageGetTimeOfFlight(&t4, id2);
	// 	tof34 = tdoaStorageGetTimeOfFlight(&t4, id3);
	// }
    // printf("tof23=%lld, tof24 = %lld, tof34 = %lld\n",tof23,tof24, tof34);
	// if((tof23 != 0) && (tof24 != 0) && (tof34 != 0)){
	// 	D12 = tdoaTriad[0].distance, D13 = tdoaTriad[1].distance, D14 = tdoaTriad[2].distance;
	// 	D23 = SPEED_OF_LIGHT * tof23 / UWB_TS_FREQ;
	// 	D24 = SPEED_OF_LIGHT * tof24 / UWB_TS_FREQ;
	// 	D34 = SPEED_OF_LIGHT * tof34 / UWB_TS_FREQ;

	// 	distances[0][0] = D12, distances[0][1] = D13, distances[0][2] = D14;
	// 	distances[1][1] = D23,distances[1][2] = D24;
	// 	distances[2][2] = D34;

	// 	printf("D12 = %f,D13 = %f,D14 = %f, D23 = %f, D24 = %f, D34 = %f\n",D12,D13,D14,D23,D24,D34);
	// 	return true;
	// }
	// return false;

//
//	for(i = 0; i < 16; i++){
//		if(anchorStorage[i].isInitialized){
//			printf("storaged anchorCtx.id is %u\n",anchorStorage[i].id);
//		}
//	}
}

/*
 * 建立内坐标轴。
 */
void createInnerAxis()
{
    float *x2 = &innerAxisQuad[1].x;
    float *x3 = &innerAxisQuad[2].x, *y3 = &innerAxisQuad[2].y;
    float *x4 = &innerAxisQuad[3].x, *y4 = &innerAxisQuad[3].y, *z4 = &innerAxisQuad[3].z;
    
    float D12, D13, D14, D23, D24, D34;
    D12 = distances[0][0],D13 = distances[0][1],D14 = distances[0][2];
    D23 = distances[1][1],D24 = distances[1][2];
    D34 = distances[2][2];

    const float pD12 = powf(D12,2), pD13 = powf(D13,2), pD14 = powf(D14,2);

    *x2 = D12;
    *x3 = (powf(D23,2)-pD13-pD12) / (-2*D12);
    *y3 = sqrtf(pD13 - powf(*x3, 2));
    *x4 = (powf(D24,2)-pD14-pD12) / (-2*D12);
    *y4 = ((-2*(*x3)+2*D12)*(*x4)+powf(*x3,2)+powf(*y3,2)-pD12-powf(D34,2)+powf(D24,2)) / (2*(*y3));
    *z4 = sqrtf(pD14 - powf(*x4,2) - powf(*y4,2));
    
    //判断从基站4的z轴坐标正负性。
    *z4 = judgeZAxis() ? *z4 : -(*z4);
    printf("anchor2(%f,0,0),anchor3(%f,%f,0),anchor4(%f,%f,%f)\n",*x2,*x3,*y3,*x4,*y4,*z4);
}

/*
 * 计算标签在内坐标轴下的坐标。
 */
void calcTagInnerCoodinate1()
{
    float R21, R31, R41;
    R21 = tdoaTriad[0].distanceDiff;
    R31 = tdoaTriad[1].distanceDiff;
    R41 = tdoaTriad[2].distanceDiff;
    
    const float x2 = innerAxisQuad[1].x;
    const float x3 = innerAxisQuad[2].x, y3 = innerAxisQuad[2].y;
    const float x4 = innerAxisQuad[3].x, y4 = innerAxisQuad[3].y, z4 = innerAxisQuad[3].z;

    const float px2 = powf(x2, 2), px3 = powf(x3, 2), py3 = powf(y3, 2);
    const float pR21 = powf(R21, 2);

    float g, h, k, l;
    
    g = (R31*x2/R21 - x3)/y3;
    h = (px3+py3-powf(R31,2)+R31*R21*(1-px2/pR21))/(2*y3);
    k = (R41*x2*y3-R21*x4*y3-R31*x2*y4+R31*x3*y4)/(R21*y3*z4);
    l = -y4*(px3+py3)/(2*y3*z4)
        +(R31*y4-R41*y3)*(px2-R21*(R21-R31))/(2*R21*y3*z4)
        +(powf(x4,2)+powf(y4,2)+powf(z4,2))/(2*z4);
    
    float d, e, f;

    d = 4*pR21*(1+powf(g,2)+powf(k,2))-4*px2;
    e = 8*pR21*(g*h+l*k)-4*x2*(pR21-px2);
    f = 4*pR21*(powf(h,2)+powf(l,2))-powf(pR21-px2,2);

    float *x, *y, *z, tmp1,tmp2;
    tmp1 = (-e+sqrtf(powf(e,2)-4*d*f))/(2*d);
    tmp2 = (-e-sqrtf(powf(e,2)-4*d*f))/(2*d);
    x = &innerAxisQuad[4].x, y = &innerAxisQuad[4].y, z = &innerAxisQuad[4].z;
    // *x = (getMinXAxis() <= tmp1 && tmp1 <= getMaxXAxis()) ? tmp1 : tmp2;
    *x = tmp1;
    *y = g*(*x)+h;
    *z = k*(*x)+l;
    printf("tag innerCoodinate(%f,%f,%f\n",*x,*y,*z);
}

/*
 * 计算标签在内坐标轴下的坐标。
 */
void calcTagInnerCoodinate2()
{
    float R21, R31, R41;
    R21 = tdoaTriad[0].distanceDiff;
    R31 = tdoaTriad[1].distanceDiff;
    R41 = tdoaTriad[2].distanceDiff;
    
    const float x2 = innerAxisQuad[1].x;
    const float x3 = innerAxisQuad[2].x, y3 = innerAxisQuad[2].y;
    const float x4 = innerAxisQuad[3].x, y4 = innerAxisQuad[3].y, z4 = innerAxisQuad[3].z;

    const float px2 = powf(x2, 2), px3 = powf(x3, 2), py3 = powf(y3, 2);
    const float pR21 = powf(R21, 2);

    float g, h, k, l;
    
    g = (R31*x2/R21 - x3)/y3;
    h = (px3+py3-powf(R31,2)+R31*R21*(1-px2/pR21))/(2*y3);
    k = (R41*x2*y3-R21*x4*y3-R31*x2*y4+R31*x3*y4)/(R21*y3*z4);
    l = -y4*(px3+py3)/(2*y3*z4)
        +(R31*y4-R41*y3)*(px2-R21*(R21-R31))/(2*R21*y3*z4)
        +(powf(x4,2)+powf(y4,2)+powf(z4,2))/(2*z4);
    
    float d, e, f;

    d = 4*pR21*(1+powf(g,2)+powf(k,2))-4*px2;
    e = 8*pR21*(g*h+l*k)-4*x2*(pR21-px2);
    f = 4*pR21*(powf(h,2)+powf(l,2))-powf(pR21-px2,2);

    float *x, *y, *z, tmp1,tmp2;
    tmp1 = (-e+sqrtf(powf(e,2)-4*d*f))/(2*d);
    tmp2 = (-e-sqrtf(powf(e,2)-4*d*f))/(2*d);
    x = &innerAxisQuad[4].x, y = &innerAxisQuad[4].y, z = &innerAxisQuad[4].z;
    // *x = (getMinXAxis() <= tmp1 && tmp1 <= getMaxXAxis()) ? tmp1 : tmp2;
    *x = tmp2;
    *y = g*(*x)+h;
    *z = k*(*x)+l;
    printf("tag innerCoodinate(%f,%f,%f\n",*x,*y,*z);
}

/*
 * 坐标轴转换（内坐标轴转换为外坐标轴）。
 */
void changeAxisFromInnerToOuter(point_t* pTagCrd)
{
    float b[3];
    float *b1 = &b[0], *b2 = &b[1], *b3 = &b[2];
    
    *b1 = innerAxisQuad[4].x;
    *b2 = multipleVector(&innerAxisQuad[4], &innerAxisQuad[2]) 
            / multipleVector(&innerAxisQuad[2], &innerAxisQuad[2]);
    *b3 = multipleVector(&innerAxisQuad[4], &innerAxisQuad[3]) 
            / multipleVector(&innerAxisQuad[3], &innerAxisQuad[3]);

    point_t u1, u2, u3;

    createVector(&tdoaTriad[0].anchorPosition[0], &tdoaTriad[0].anchorPosition[1], &u1);
    createVector(&tdoaTriad[0].anchorPosition[0], &tdoaTriad[1].anchorPosition[1], &u2);
    createVector(&tdoaTriad[0].anchorPosition[0], &tdoaTriad[2].anchorPosition[1], &u3);
    
    float M[3][3] = {u1.x,u1.y,u1.z,u2.x,u2.y,u2.z,u3.x,u3.y,u3.z};
    float array[3] = {0};
    int i, j;
    
    for(i = 0; i < 3; i++){
        for(j = 0; j < 3; j++){
            array[i] += b[j]*M[j][i];
        }
    }
    pTagCrd->x = array[0], pTagCrd->y = array[1], pTagCrd->z = array[2];
    pTagCrd->timestamp = clock_time();
    printf("tag outer Coodinate(%f,%f,%f)\n",array[0],array[1],array[2]);
}

static int getAnchorIdIndex(uint8_t id)
{
    int i;
    for(i = 0; i < TDOA_ANCHOR_COUNT; i++){
        if(ids[i] == id)
            return i;   
    }
    return -1;
}

static int createAnchorIdIndex(uint8_t id)
{
    int i;
    for(i = 0; i < TDOA_ANCHOR_COUNT; i++){
        if(ids[i] == 0){
        	ids[i] = id;
        	return i;
        }
    }
    return -1;
}

static void createTdoaMeasurement(tdoaMeasurement_t* src, tdoaMeasurement_t* dst)
{
    setAnchorPosition(&src->anchorPosition[0], &dst->anchorPosition[0]);
    setAnchorPosition(&src->anchorPosition[1], &dst->anchorPosition[1]);
    dst->distanceDiff = src->distanceDiff;
    dst->distance = src->distance;
    dst->idA = src->idA;
    dst->idB = src->idB;
    dst->endOfLife = src->endOfLife;
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

    n.x = v12.y*v13.z-v13.y*v12.z;
    n.y = v13.x*v12.z-v12.x*v13.z;
    n.z = v12.x*v13.y-v13.x*v12.y;

    float cosVal;
    cosVal = multipleVector(&n, &v14) / sqrtf(multipleVector(&n, &n)*multipleVector(&v14, &v14));

    return (cosVal > 0) ? true : false;
}

static float getMinXAxis()
{
    int i;
    float res = 0;
    for(i = 1; i < 4; i++){
        if(innerAxisQuad[i].x < res)
            res = innerAxisQuad[i].x;
    }
    return res;
}

static float getMaxXAxis()
{
    int i;
    float res = 0;
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

static float multipleVector(point_t* v1, point_t* v2)
{
    return (v1->x)*(v2->x)+(v1->y)*(v2->y)+(v1->z)*(v2->z);
}

// static bool judgeZAxis()
// {
//     point_t vector12, vector13, n, vector14;
//     float cosVal;

//     createVector(&innerAxisQuad[0], &innerAxisQuad[1], &vector12);
//     createVector(&innerAxisQuad[0], &innerAxisQuad[2], &vector13);
//     createVector(&innerAxisQuad[0], &innerAxisQuad[3], &vector14);

//     n.x = 1;
//     n.y = (vector12.x*vector13.z - vector13.x*vector12.z) 
//             / (vector13.y*vector12.z - vector12.y*vector13.z);
//     n.z = (-vector12.x - vector12.y*n.y) / vector12.z;

//     cosVal = multipleVector(&n, &vector14) / sqrtf(multipleVector(&n, &n)
//             *multipleVector(&vector14, &vector14));
    
//     //是否有可能四个点都在一个平面上。
//     return cosVal > 0 ? true : false;
// }
