#ifndef _ALGO_DATA_LIMITING_H__
#define _ALGO_DATA_LIMITING_H__

#include "stdint.h"

/**
 * @brief 绝对限制
 */
extern void rflAbsLimit(float *num, float limit);

/**
 * @brief 单精度浮点型限幅函数
 */
extern float rflFloatConstrain(float input, float min_value, float max_value);

/**
 * @brief 单精度浮点型循环限幅函数
 */
extern float rflFloatLoopConstrain(float input, float min_value, float max_value);

#endif /* _ALGO_DATA_LIMITING_H__ */
