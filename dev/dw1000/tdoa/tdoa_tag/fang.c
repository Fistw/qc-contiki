#include "fang.h"
#include "tdoa_tag_storage.h"

static tdoaQueue_t queue;

static uint8_t ids[TDOA_ANCHOR_COUNT];
static uint8_t packets[TDOA_ANCHOR_COUNT];

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
    return (m.distance > m.distanceDiff);
}

/*
 * 将收到的tdoaMeasurement数据放入数组，更新标签从相应基站收包数。
 */
int fangPutTdoaMeasurement(tdoaMeasurement_t* measure)
{ 
    int sameSlot = -1, firstEmptySlot = -1, expiredSlot = 0;
    uint8_t idA = measure->idA, idB = measure->idB;
    uint32_t expiredTime = clock_time();
    int i;

    for(i = 0; i < TDOA_QUEUE_LENTH; i++){
        if(!(queue[i].used & 1)){
            if(firstEmptySlot != -1)
                firstEmptySlot = i;
        }else if(queue[i].endOfLife < expiredTime){
            queue[i].used &= 0;
            expiredSlot = i;

            //同时更新ids数组及packets数组。
            int idxC, idxD;
            if((idxC = getAnchorIdIndex(queue[i].idA)) != -1)
                packets[idxC]--;
            else
                printf("Error: getAnchorIdIndex return -1.\n");

            if((idxD = getAnchorIdIndex(queue[i].idB)) != -1)
                packets[idxD]--;
            else
                printf("Error: getAnchorIdIndex return -1.\n");
        }else if((queue[i].idA == idA) && (queue[i].idB == idB)){
            sameSlot = i;
        }
    }

    if(sameSlot != -1){
        createTdoaMeasurement(measure, &queue[sameSlot]);
    }else{
        if(firstEmptySlot != -1){
            createTdoaMeasurement(measure, &queue[firstEmptySlot]);
            queue[firstEmptySlot].used |= 1;
        }else{
            createTdoaMeasurement(measure, &queue[expiredSlot]);
            queue[expiredSlot].used |= 1;
        }

        //同时更新ids数组及packets数组。
        int idxA, idxB;
        if((idxA = getAnchorIdIndex(idA)) == -1){
            idxA = createAnchorIdIndex(idA);
            ids[idxA] = idA;
        }else{
            printf("Error: createAnchorIdIndex return -1.\n");
        }
        if((idxB = getAnchorIdIndex(idB)) == -1){
            idxB = createAnchorIdIndex(idB);
            ids[idxB] = idB;
        }else{
            printf("Error: createAnchorIdIndex return -1.\n");
        }
        packets[idxA]++;
        packets[idxB]++;

        return (packets[idxA] >= 3) ? idxA : ((packets[idxB] >= 3) ? idxB : -1);
    } 
    return -1;
}

/*
 * 从队列中取出三个TDOA数据，一个主基站，三个从基站。
 */
void fangGetTdoaMeasurement(int idx)
{
    uint8_t id = ids[idx];
    int i, count = 0;
    for(i = 0; i < TDOA_QUEUE_LENTH, count < 3; i++){
        if((queue[i].used &= 1) && (queue[i].idA == id || queue[i].idB == id)){
            createTdoaMeasurement(&queue[i], &tdoaTriad[count]);
            if(queue[i].idB == id)
                inverseTdoaMeasurement(&tdoaTriad[count]);
            count++;
        }
    }
}

/*
 * 获取基站测距。
 */
bool getAnchorDistances(tdoaAnchorInfo_t anchorStorage[])
{
    const uint8_t id2, id3, id4;
    tdoaAnchorContext_t t2, t3;
    uint32_t nowms = clock_time();
    int64_t tof23, tof24, tof34;
    float D23, D24, D14, D23, D24, D34;
    
    id2 = tdoaTriad[0].idB, id3 = tdoaTriad[1].idB, id4 = tdoaTriad[2].idB;
    
    if(tdoaStorageGetAnchorCtx(anchorStorage, id2, nowms, &t2)
            && tdoaStorageGetAnchorCtx(anchorStorage, id3, nowms, &t3)){
        tof23 = tdoaStorageGetTimeOfFlight(&t2, id3);
        tof24 = tdoaStorageGetTimeOfFlight(&t2, id4);
        tof34 = tdoaStorageGetTimeOfFlight(&t3, id4);

        D23 = tdoaTriad[0].distance, D24 = tdoaTriad[1].distance, D14 = tdoaTriad[2].distance; 
        D23 = SPEED_OF_LIGHT * tof23 / UWB_TS_FREQ;
        D24 = SPEED_OF_LIGHT * tof24 / UWB_TS_FREQ;
        D34 = SPEED_OF_LIGHT * tof34 / UWB_TS_FREQ;

        distances[0][0] = D23, distances[0][1] = D24, distance[0][2] = D14;
        distances[1][1] = D23,distances[1][2] = D24;
        distances[2][2] = D34;

        return true;
    }
    return false;
}

/*
 * 建立内坐标轴。
 */
void createInnerAxis()
{
    float *x2 = &innerAxisQuad[1].x;
    float *x3 = &innerAxisQuad[2].x, *y3 = &innerAxisQuad[2].y;
    float *x4 = &innerAxisQuad[3].x, *y4 = &innerAxisQuad[3].y, *z4 = &innerAxisQuad[3].z;
    
    const float D12, D13, D14, D23, D24, D34;
    D12 = distance[0][0],D13 = distance[0][1],D14 = distance[0][2];
    D23 = distance[1][1],D24 = distance[1][2];
    D34 = distance[2][2];

    const float pD12 = powf(D12,2), pD13 = powf(D13,2), pD14 = powf(D14,2);

    *x2 = D12;
    *x3 = (powf(D23,2)-pD13-pD12) / (-2*D12);
    *y3 = sqrtf(pD13 - powf(*x3, 2));
    *x4 = (powf(D24,2)-pD14-pD12) / (-2*D12);
    *y4 = (-2*(*x3)+2*D12)*(*x4)+powf(*x3,2)+powf(*y3,2)-pD12-powf(D34,2)+powf(D24,2))
                / (2*(*y3)+1);
    *z4 = sqrtf(pD14 - powf(*x4,2) - powf(*y4,2));
    
    //判断从基站4的z轴坐标正负性。
    *z4 = judgeZAxis() ? *z4 : -(*z4);
}

/*
 * 计算标签在内坐标轴下的坐标。
 */
void calcTagInnerCoodinate()
{
    const float D12, D13, D14, D23, D24, D34;
    D12 = distance[0][0],D13 = distance[0][1],D14 = distance[0][2];
    D23 = distance[1][1],D24 = distance[1][2];
    D34 = distance[2][2];
    
    const float R21, R31, R41;
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
    f = 4*pR21*(powf(h,2)+powf(l,2))-powf(pR21-px2,2)

    float *x, *y, *z, tmp1,tmp2;
    tmp1 = (-e+sqrtf(powf(powf(e,2)-4*d*f)))/(2*d);
    tmp2 = (-e-sqrtf(powf(powf(e,2)-4*d*f)))/(2*d);
    x = &innerAxisQuad[4].x, y = &innerAxisQuad[4].y, z = &innerAxisQuad[4].z;
    *x = getMinXAxis() <= tmp1 <= getMaxXAxis() ? tmp1 : tmp2;
    *y = g*(*x)+h;
    *z = k*(*x)+l;
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
        if(packets[i] == 0)
            return i;
    }
    return -1;
}

static void createTdoaMeasurement(tdoaMeasurement_t* src, tdoaMeasurement_t* dst)
{
    setAnchorPosition(&src.anchorPosition[0], &dst.anchorPosition[0]);
    setAnchorPosition(&src.anchorPosition[1], &dst.anchorPosition[1]);
    dst.distanceDiff = src.distanceDiff;
    dst.distance = src.distance;
    dst.idA = src.idA;
    dst.idB = src.idB;
    dst.endOfLife = src.endOfLife;
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
    return (tdoaTriad[2].anchorPosition[1].z > 0);
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