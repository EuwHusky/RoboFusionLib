#include "algo_data_limiting.h"

/**
 * @brief 绝对限制
 */
void rflAbsLimit(float *num, float limit)
{
    if (*num > limit)
    {
        *num = limit;
    }
    else if (*num < -limit)
    {
        *num = -limit;
    }
}

/**
 * @brief UINT32限幅函数
 */
uint32_t rflUint32Constrain(uint32_t input, uint32_t min_value, uint32_t max_value)
{
    if (max_value < min_value)
    {
        return input;
    }

    if (input > max_value)
    {
        return max_value;
    }
    else if (input < min_value)
    {
        return min_value;
    }

    return input;
}

/**
 * @brief 单精度浮点型限幅函数
 */
float rflFloatConstrain(float input, float min_value, float max_value)
{
    if (max_value < min_value)
    {
        return input;
    }

    if (input > max_value)
    {
        return max_value;
    }
    else if (input < min_value)
    {
        return min_value;
    }

    return input;
}

/**
 * @brief 单精度浮点型循环限幅函数
 */
float rflFloatLoopConstrain(float input, float min_value, float max_value)
{
    if (max_value < min_value)
    {
        return input;
    }

    if (input > max_value)
    {
        float len = max_value - min_value;
        while (input > max_value)
        {
            input -= len;
        }
    }
    else if (input < min_value)
    {
        float len = max_value - min_value;
        while (input < min_value)
        {
            input += len;
        }
    }
    return input;
}
