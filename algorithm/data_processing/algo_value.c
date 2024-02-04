#include "algo_value.h"

/**
 * @brief 采用浮点数据等比例转换成整数
 * @param x_int 要转换的无符号整数
 * @param x_min 目标浮点数的最小值
 * @param x_max 目标浮点数的最大值
 * @param bits 无符号整数的位数
 */
float rflUintToFloat(uint32_t x_int, float x_min, float x_max, uint32_t bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * @brief 将浮点数转换为无符号整数
 * @param x 要转换的浮点数
 * @param x_min 浮点数的最小值
 * @param x_max 浮点数的最大值
 * @param bits 无符号整数的位数
 */

uint32_t rflFloatToUint(float x, float x_min, float x_max, uint32_t bits)
{
    /// Converts a float to an unsigned int, given range and number of bits///
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
