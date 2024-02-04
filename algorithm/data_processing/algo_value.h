#ifndef _ALGO_VALUE_H__
#define _ALGO_VALUE_H__

#include "stdint.h"

/**
 * @brief 采用浮点数据等比例转换成整数
 * @param x_int 要转换的无符号整数
 * @param x_min 目标浮点数的最小值
 * @param x_max 目标浮点数的最大值
 * @param bits 无符号整数的位数
 */
extern float rflUintToFloat(uint32_t x_int, float x_min, float x_max, uint32_t bits);

/**
 * @brief 将浮点数转换为无符号整数
 * @param x 要转换的浮点数
 * @param x_min 浮点数的最小值
 * @param x_max 浮点数的最大值
 * @param bits 无符号整数的位数
 */

extern uint32_t rflFloatToUint(float x, float x_min, float x_max, uint32_t bits);

#endif /* _ALGO_VALUE_H__ */
