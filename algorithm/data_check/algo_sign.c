#include "algo_sign.h"

// 判断符号位
float rflJudgeFloatSign(float value)
{
    if (value >= 0.0f)
    {
        return 1.0f;
    }
    else
    {
        return -1.0f;
    }
}

// 将0转为1，非0转为-1
float rflConvertTfToSign(uint8_t value)
{
    return value ? -1.0f : 1.0f;
}
