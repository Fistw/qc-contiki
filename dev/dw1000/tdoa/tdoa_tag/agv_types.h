#ifndef __AGV_TYPES_H__
#define __AGV_TYPES_H__

#include <stdint.h>
#include <stdbool.h>

/* x,y,z vector */
struct vec3_s
{
    uint32_t timestamp; // Timestamp when the data was computed

    float x;
    float y;
    float z;
};

typedef struct vec3_s point_t;

typedef struct tdoaMeasurement_s
{
    point_t anchorPosition[2];
    float distanceDiff;
    float stdDev;
} tdoaMeasurement_t;

#endif