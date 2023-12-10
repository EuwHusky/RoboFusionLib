#ifndef _ALGO_FILTER_H__
#define _ALGO_FILTER_H__

#include "stdint.h"

#define __packed __attribute__((packed))

typedef struct
{
    float input;        // 输入数据
    float out;          // 输出数据
    float min_value;    // 限幅最小值
    float max_value;    // 限幅最大值
    float frame_period; // 时间间隔
} __packed ramp_function_source_t;

typedef struct
{
    float input;        // 输入数据
    float out;          // 滤波输出的数据
    float num[1];       // 滤波参数
    float frame_period; // 滤波的时间间隔 单位 s
} __packed first_order_filter_type_t;

typedef struct
{
    float input;  // 输入数据
    float output; // 滤波输出的数据
    float *data_flow;
    uint8_t window_depth; // 窗口深度
    uint8_t data_depth;   // 数据深度
    uint8_t wall_depth;   // 墙壁深度

    uint8_t status;
} __packed sliding_window_filter_s_t;

/**
 * @brief          斜波函数初始化
 * @author         RM
 * @param[in]      斜波函数结构体
 * @param[in]      间隔的时间，单位 s
 * @param[in]      最大值
 * @param[in]      最小值
 * @retval         返回空
 */
extern void rlfRampInit(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min);
/**
 * @brief          斜波函数计算，根据输入的值进行叠加， 输入单位为 /s 即一秒后增加输入的值
 * @author         RM
 * @param[in]      斜波函数结构体
 * @param[in]      输入值
 * @param[in]      滤波参数
 * @retval         返回空
 */
extern void rlfRampCalc(ramp_function_source_t *ramp_source_type, float input);
/**
 * @brief          一阶低通滤波初始化
 * @author         RM
 * @param[in]      一阶低通滤波结构体
 * @param[in]      间隔的时间，单位 s
 * @param[in]      滤波参数
 * @retval         返回空
 */
extern void rlfFirstOrderFilterInit(first_order_filter_type_t *first_order_filter_type, float frame_period,
                                    const float num[1]);
/**
 * @brief          一阶低通滤波计算
 * @author         RM
 * @param[in]      一阶低通滤波结构体
 * @param[in]      间隔的时间，单位 s
 * @retval         返回空
 */
extern void rlfFirstOrderFilterCali(first_order_filter_type_t *first_order_filter_type, float input);
/**
 * @name rlfSlidingWindowFilterInit
 * @brief 滑动窗口滤波初始化
 * @param sliding_window_filter 滑动窗口滤波结构体
 * @param data_depth 数据宽度
 * @param wall_depth 墙壁宽度
 * @return uint8_t
 */
extern uint8_t rlfSlidingWindowFilterInit(sliding_window_filter_s_t *sliding_window_filter, const uint8_t data_depth,
                                          const uint8_t wall_depth);
/**
 * @name rlfSlidingWindowFilterCalc
 * @brief 滑动窗口滤波计算
 * @param sliding_window_filter 滑动窗口滤波结构体
 * @param input 输入的数据
 * @return float
 */
extern float rlfSlidingWindowFilterCalc(sliding_window_filter_s_t *sliding_window_filter, float input);

#endif /* _ALGO_FILTER_H__ */
