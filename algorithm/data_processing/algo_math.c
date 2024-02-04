#include "algo_math.h"

float rflSqrt(float x)
{
    const float threehalfs = 1.5f;

    float xhalf = x * 0.5F;
    int i = *(int *)&x;
    i = 0x5f375a86 - (i >> 1);
    float y = *(float *)&i;
    y = y * (threehalfs - (xhalf * y * y));
    y = y * (threehalfs - (xhalf * y * y));
    y = y * (threehalfs - (xhalf * y * y));
    return x * y;
}
