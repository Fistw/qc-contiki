#include "sensors.h"

bool sensorsReadGyro(Axis3f *gyro)
{
    int i;
    for(i = 0; i < 3; i++)
        gyro->axis[i] = 0;
    return true;
}

bool sensorsReadAcc(Axis3f *acc)
{
    int i;
    for(i = 0; i < 3; i++)
        acc->axis[i] = 0;
    return true;
}