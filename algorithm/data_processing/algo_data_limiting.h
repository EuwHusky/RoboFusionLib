#ifndef _ALGO_DATA_LIMITING_H__
#define _ALGO_DATA_LIMITING_H__

#include "stdint.h"

/**
 * @brief 死区置零
 */
#define rflDeadZoneZero(input, dealine) (((input) > (dealine) || (input) < -(dealine)) ? (input) : 0)

/**
 * @brief 绝对限制
 */
extern void rflAbsLimit(float *num, float limit);

/**
 * @brief UINT32限幅函数
 */
extern uint32_t rflUint32Constrain(uint32_t input, uint32_t min_value, uint32_t max_value);

/**
 * @brief 单精度浮点型限幅函数
 */
extern float rflFloatConstrain(float input, float min_value, float max_value);

/**
 * @brief 单精度浮点型循环限幅函数
 */
extern float rflFloatLoopConstrain(float input, float min_value, float max_value);

/**
 * @brief 单精度浮点型求圆上劣弧中点位置
 */
extern float rflFloatCircleMidPoint(float point_0, float point_1, float min_value, float max_value);

#endif /* _ALGO_DATA_LIMITING_H__ */
