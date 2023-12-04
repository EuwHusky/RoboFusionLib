#include "algo_data_limiting.h"

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
