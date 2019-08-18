#include "taylor.h"
#include "arm_math.h"
#include <math.h>
#include <stdbool.h>

static void createVector(point_t* pstart, point_t* pend, point_t* vector);
static float multipleVector(point_t* v1, point_t* v2);

static void createMatrixAandb(tdoaMeasurement_t* tdoa, point_t* pTagCrd, float* argsA, float* argsb);

float calcMinfuncF(tdoaMeasurement_t* tdoa, point_t* pTagCrd);
float oneSearch(tdoaMeasurement_t* tdoa, point_t* pTagCrd, float* argsd);

static void printFlag(arm_status s);

/*
 * taylor算法
 */
void taylor(tdoaMeasurement_t* tdoa, point_t* pTagCrd)
{
    float argsA[3][2], argsb[3], argsd[2];
    arm_matrix_instance_f32 A = {.numRows = 3,
                                 .numCols = 2,
                                 .pData = argsA};
    arm_matrix_instance_f32 b = {.numRows = 3,
                                 .numCols = 1,
                                 .pData = argsb};
    arm_matrix_instance_f32 d = {.numRows = 2,
                                 .numCols = 1,
                                 .pData = argsd};
    float tmp1a[2][3];
    arm_matrix_instance_f32 tmp1m = {.numRows = 2,
                                     .numCols = 3,
                                     .pData = tmp1a};
    float tmp2a[2][2];
    arm_matrix_instance_f32 tmp2m = {.numRows = 2,
                                     .numCols = 2,
                                     .pData = tmp2a};
    float tmp3a[2][2];
    arm_matrix_instance_f32 tmp3m = {.numRows = 2,
                                     .numCols = 2,
                                     .pData = tmp3a};      
    float tmp4a[2][3];
    arm_matrix_instance_f32 tmp4m = {.numRows = 2,
                                     .numCols = 3,
                                     .pData = tmp4a};        
    float nmd;                                                           
    do{
        for(int i = 0; i < 3; i++)
            createMatrixAandb(tdoa+i, pTagCrd, argsA+i, argsb+i);
        arm_status flag;
        if(flag = arm_mat_trans_f32(&A, &tmp1m)){
            printf("Execute trans(A) error,");
            printFlag(flag);
            printf("\n");
        }

        if(flag = arm_mat_mult_f32(&tmp1m, &A, &tmp2m)){
            printf("Execute mult(At,A) error,");
            printFlag(flag);
            printf("\n");            
        }

        if(flag = arm_mat_inverse_f32(&tmp2m, &tmp3m)){
            printf("Execute inverse(At*A) error,");
            printFlag(flag);
            printf("\n");
        }

        if(flag = arm_mat_mult_f32(&tmp3m, &tmp1m, &tmp4m)){
            printf("Execute mult((At*A)-1,At) error,");
            printFlag(flag);
            printf("\n");
        }

        if(flag = arm_mat_mult_f32(&tmp4m, &b, &d)){
            printf("Execute mult((At*A)-1*At,b) error,");
            printFlag(flag);
            printf("\n");
        }
        nmd = oneSearch(tdoa, pTagCrd, argsd);
        pTagCrd->x += nmd*argsd[0];
        pTagCrd->y += nmd*argsd[1];
    }while(fabs(nmd*argsd[0])+fabs(nmd*argsd[1]) < 1e-2);
}

void createMatrixAandb(tdoaMeasurement_t* tdoa, point_t* pTagCrd, float* argsA, float* argsb)
{
    float *a,*b,*c,R1,R2;
    a=argsA, b=argsA+1, c=argsb;
    point_t v;
    createVector(tdoa->anchorPosition, pTagCrd, &v);
    R1 = sqrtf(multipleVector(&v,&v));
    createVector(tdoa->anchorPosition+1, pTagCrd, &v);
    R2 = sqrtf(multipleVector(&v,&v));

    *a = (pTagCrd->x-tdoa->anchorPosition[1].x)/R2-(pTagCrd->x-tdoa->anchorPosition[0].x)/R1;
    *b = (pTagCrd->y-tdoa->anchorPosition[1].y)/R2-(pTagCrd->y-tdoa->anchorPosition[0].y)/R1;
    *c = R2-R1-tdoa->distanceDiff;
}

float oneSearch(tdoaMeasurement_t* tdoa, point_t* pTagCrd, float* argsd)
{
    float l,r,nmd1,nmd2;
    l = -1,r = 1;
    nmd1 = l+0.382*(r-l),nmd2 = l+0.618*(r-l);
    int count = 1;
    do{
        float funcF1, funcF2;
        funcF1 = funcF2 = 0;
        point_t tagCrd1 = {.x = pTagCrd->x+nmd1*argsd[0],
                           .y = pTagCrd->y+nmd1*argsd[1],
                           .z = pTagCrd->z};
        point_t tagCrd2 = {.x = pTagCrd->x+nmd2*argsd[0],
                           .y = pTagCrd->y+nmd2*argsd[1],
                           .z = pTagCrd->z};
        for(int i = 0; i < 3; i++){
            funcF1 += calcfuncF(tdoa+i, &tagCrd1);
            funcF2 += calcfuncF(tdoa+i, &tagCrd2);
        }
        printf("%dth iteration: r-l=%f, l=%f, nmd1=%f, "
               "nmd2=%f, r=%f, funcF1=%f, funcF2=%f\n",count,r-l,l,nmd1,nmd2,r,funcF1,funcF2);
        if(fabs(funcF1 - funcF2) < 1e-2){
            l = nmd1,r = nmd2;
            break;
        }else if(funcF1 > funcF2){
            l = nmd1;
            nmd1 = nmd2;
            nmd2 = l+0.618*(r-l);
        }else{
            r = nmd2;
            nmd2 = nmd1;
            nmd1 = l+0.382*(r-l);
        }
        count++;
    }while(r-l > 0.01);
    return (r-l)/2;
}

float calcMinfuncF(tdoaMeasurement_t* tdoa, point_t* pTagCrd)
{
    float x1,y1,z1,x2,y2,z2,x,y,z;

    x1 = tdoa->anchorPosition[0].x,y1 = tdoa->anchorPosition[0].y,z1 = tdoa->anchorPosition[0].z;
    x2 = tdoa->anchorPosition[1].x,y2 = tdoa->anchorPosition[1].y,z2 = tdoa->anchorPosition[1].z;
    x = pTagCrd->x,y = pTagCrd->y,z = pTagCrd->z;

    return powf(sqrtf(powf(x2-x,2)+powf(y2-y,2)+powf(z2-z,2))
               -sqrtf(powf(x1-x,2)+powf(y1-y,2)+powf(z1-z,2))
               -tdoa->distanceDiff,2);
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

void printFlag(arm_status s)
{
    switch(s){
    case ARM_MATH_SUCCESS:
        break;
    case ARM_MATH_ARGUMENT_ERROR:       /**< One or more arguments are incorrect */
        printf("arm_math_argument_error");
        break;
    case ARM_MATH_LENGTH_ERROR:         /**< Length of data buffer is incorrect */
        printf("arm_math_length_error");
        break;
    case ARM_MATH_SIZE_MISMATCH:        /**< Size of matrices is not compatible with the operation. */
        printf("arm_math_size_mismatch");
        break;
    case ARM_MATH_NANINF:               /**< Not-a-number (NaN) or infinity is generated */
        printf("arm_math_naninf");
        break;
    case ARM_MATH_SINGULAR:             /**< Generated by matrix inversion if the input matrix is singular and cannot be inverted. */
        printf("arm_math_singular");
        break;
    default:
        printf("don't know error");
        break;
    }
}