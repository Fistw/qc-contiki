#ifndef __AGV_TYPES_H__
#define __AGV_TYPES_H__

#include <stdint.h>
#include <stdbool.h>

// imu_types.h 中的数值类型定义
typedef union {
    struct
    {
        float x;
        float y;
        float z;
    };
    float axis[3];
} Axis3f;

/* Data structure used by the subsystem.
 * All have a timestamp to be set when the data is calculated.
 */

/** Attitude in euler angle form */
typedef struct attitude_s
{
    uint32_t timestamp; // Timestamp when the data was computed

    float roll;
    float pitch;
    float yaw;
} attitude_t;

/* x,y,z vector */
struct vec3_s
{
    uint32_t timestamp; // Timestamp when the data was computed

    float x;
    float y;
    float z;
};

typedef struct vec3_s vector_t;
typedef struct vec3_s point_t;
typedef struct vec3_s velocity_t;
typedef struct vec3_s acc_t;

/* Orientation as a quaternion */
typedef struct quaternion_s
{
    uint32_t timestamp;

    union {
        struct
        {
            float q0;
            float q1;
            float q2;
            float q3;
        };
        struct
        {
            float x;
            float y;
            float z;
            float w;
        };
    };
} quaternion_t;

typedef struct tdoaMeasurement_s
{
    point_t anchorPosition[2];
    float distanceDiff;
    float stdDev;
} tdoaMeasurement_t;

typedef struct baro_s
{
    float pressure;    // mbar
    float temperature; // degree Celcius
    float asl;         // m (ASL = altitude above sea level)
} baro_t;

typedef struct zDistance_s
{
    uint32_t timestamp;
    float distance; // m
} zDistance_t;

typedef struct positionMeasurement_s
{
    union {
        struct
        {
            float x;
            float y;
            float z;
        };
        float pos[3];
    };
    float stdDev;
} positionMeasurement_t;

typedef struct distanceMeasurement_s
{
    union {
        struct
        {
            float x;
            float y;
            float z;
        };
        float pos[3];
    };
    float distance;
    float stdDev;
} distanceMeasurement_t;

/** Flow measurement**/
typedef struct flowMeasurement_s
{
    uint32_t timestamp;
    union {
        struct
        {
            float dpixelx; // Accumulated pixel count x
            float dpixely; // Accumulated pixel count y
        };
        float dpixel[2]; // Accumulated pixel count
    };
    float stdDevX; // Measurement standard deviation
    float stdDevY; // Measurement standard deviation
    float dt;      // Time during which pixels were accumulated
} flowMeasurement_t;

/** TOF measurement**/
typedef struct tofMeasurement_s
{
    uint32_t timestamp;
    float distance;
    float stdDev;
} tofMeasurement_t;

/** Absolute height measurement */
typedef struct heightMeasurement_s
{
    uint32_t timestamp;
    float height;
    float stdDev;
} heightMeasurement_t;

typedef struct sensorData_s
{
    Axis3f acc;  // Gs
    Axis3f gyro; // deg/s
    Axis3f mag;  // gauss
    baro_t baro;
    zDistance_t zrange;
    point_t position; // m

    // #ifdef LOG_SEC_IMU
    //     Axis3f accSec;  // Gs
    //     Axis3f gyroSec; // deg/s
    // #endif
    uint64_t interruptTimestamp;
} sensorData_t;

typedef struct state_s
{
    attitude_t attitude; // deg (legacy CF2 body coordinate system, where pitch is inverted)
    quaternion_t attitudeQuaternion;
    point_t position;    // m
    velocity_t velocity; // m/s
    acc_t acc;           // Gs (but acc.z without considering gravity)
} state_t;

typedef struct control_s
{
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    float thrust;
} control_t;

#endif