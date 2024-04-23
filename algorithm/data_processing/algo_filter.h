#ifndef _ALGO_FILTER_H__
#define _ALGO_FILTER_H__

#include "stdint.h"

typedef struct
{
    float target;       // 目标值
    float out;          // 输出数据
    float min_value;    // 限幅最小值
    float max_value;    // 限幅最大值
    float frame_period; // 时间间隔
} ramp_function_source_t;

typedef struct
{
    float input;        // 输入数据
    float out;          // 滤波输出的数据
    float num;          // 滤波参数
    float frame_period; // 滤波的时间间隔 单位 s
} first_order_filter_type_t;

typedef struct
{
    float input;  // 输入数据
    float output; // 滤波输出的数据
    float *data_flow;
    uint8_t window_depth; // 窗口深度
    uint8_t data_depth;   // 数据深度
    uint8_t wall_depth;   // 墙壁深度

    uint8_t status;
} sliding_window_filter_s_t;

/**
 * @brief          斜波函数初始化
 * @author         RM
 * @param[in]      斜波函数结构体
 * @param[in]      间隔的时间 单位 s
 * @param[in]      最大值
 * @param[in]      最小值
 * @retval         返回空
 */
extern void rflRampInit(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min);
/**
 * @brief          斜波函数计算
 * @author         RM
 * @param[in]      斜波函数结构体
 * @param[in]      变化速率 一秒内变化的值
 * @param[in]      目标值
 * @retval         斜坡函数输出值
 */
extern float rflRampCalc(ramp_function_source_t *ramp_source_type, float speed, float target);
/**
 * @brief          一阶低通滤波初始化
 * @author         RM
 * @param[in]      一阶低通滤波结构体
 * @param[in]      间隔的时间，单位 s
 * @param[in]      滤波参数
 * @retval         返回空
 */
extern void rflFirstOrderFilterInit(first_order_filter_type_t *first_order_filter_type, float frame_period,
                                    const float num);
/**
 * @brief          一阶低通滤波计算
 * @author         RM
 * @param[in]      一阶低通滤波结构体
 * @param[in]      间隔的时间，单位 s
 * @retval         返回空
 */
extern void rflFirstOrderFilterCali(first_order_filter_type_t *first_order_filter_type, float input);
/**
 * @name rflSlidingWindowFilterInit
 * @brief 滑动窗口滤波初始化
 * @param sliding_window_filter 滑动窗口滤波结构体
 * @param data_depth 数据宽度
 * @param wall_depth 墙壁宽度
 * @return uint8_t
 */
extern uint8_t rflSlidingWindowFilterInit(sliding_window_filter_s_t *sliding_window_filter, const uint8_t data_depth,
                                          const uint8_t wall_depth);
/**
 * @name rflSlidingWindowFilterCalc
 * @brief 滑动窗口滤波计算
 * @param sliding_window_filter 滑动窗口滤波结构体
 * @param input 输入的数据
 * @return float
 */
extern float rflSlidingWindowFilterCalc(sliding_window_filter_s_t *sliding_window_filter, float input);

#endif /* _ALGO_FILTER_H__ */
