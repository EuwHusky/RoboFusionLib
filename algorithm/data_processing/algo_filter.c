#include "math.h"
#include "stdlib.h"
#include "string.h"

#include "algo_filter.h"

#include "algo_data_limiting.h"

/**
 * @brief          斜波函数初始化
 * @author         RM
 * @param[in]      斜波函数结构体
 * @param[in]      间隔的时间，单位 s
 * @param[in]      最大值
 * @param[in]      最小值
 * @retval         返回空
 */
void rlfRampInit(ramp_function_source_t *ramp_source_type, float p_factor, float max, float min)
{
    ramp_source_type->p_factor = p_factor;
    ramp_source_type->max_value = max;
    ramp_source_type->min_value = min;
    ramp_source_type->input = 0.0f;
    ramp_source_type->out = 0.0f;
}

/**
 * @brief          斜波函数计算，根据输入的值进行叠加， 输入单位为 /s 即一秒后增加输入的值
 * @author         RM
 * @param[in]      斜波函数结构体
 * @param[in]      输入值
 * @param[in]      滤波参数
 * @retval         返回空
 */
float rlfRampCalc(ramp_function_source_t *ramp_source_type, float input)
{
    ramp_source_type->input = input;
    if (ramp_source_type->input > ramp_source_type->max_value)
    {
        ramp_source_type->input = ramp_source_type->max_value;
    }
    else if (ramp_source_type->input < ramp_source_type->min_value)
    {
        ramp_source_type->input = ramp_source_type->min_value;
    }

    ramp_source_type->out += (ramp_source_type->input - ramp_source_type->out) * ramp_source_type->p_factor;

    ramp_source_type->out = rflDeadZoneZero(ramp_source_type->out, 0.00001f);

    return ramp_source_type->out;
}

/**
 * @brief          一阶低通滤波初始化
 * @author         RM
 * @param[in]      一阶低通滤波结构体
 * @param[in]      间隔的时间，单位 s
 * @param[in]      滤波参数
 * @retval         返回空
 */
void rlfFirstOrderFilterInit(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1])
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0] = num[0];
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

/**
 * @brief          一阶低通滤波计算
 * @author         RM
 * @param[in]      一阶低通滤波结构体
 * @param[in]      间隔的时间，单位 s
 * @retval         返回空
 */
void rlfFirstOrderFilterCali(first_order_filter_type_t *first_order_filter_type, float input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
        first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) *
            first_order_filter_type->out +
        first_order_filter_type->frame_period /
            (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
}

/**
 * @name rlfSlidingWindowFilterInit
 * @brief 滑动窗口滤波初始化
 * @param sliding_window_filter 滑动窗口滤波结构体
 * @param data_depth 数据宽度
 * @param wall_depth 墙壁宽度
 * @return uint8_t
 */
uint8_t rlfSlidingWindowFilterInit(sliding_window_filter_s_t *sliding_window_filter, const uint8_t data_depth,
                                   const uint8_t wall_depth)
{
    if ((data_depth < (2 * wall_depth)) || (wall_depth < 1)) // 数据深度有误、墙壁宽度有误
    {
        sliding_window_filter->status = 1;
        return sliding_window_filter->status;
    }

    sliding_window_filter->data_depth = data_depth;
    sliding_window_filter->wall_depth = wall_depth;
    sliding_window_filter->window_depth = data_depth - (2 * wall_depth);

    if (sliding_window_filter->window_depth < 2) // 窗口宽度有误
    {
        sliding_window_filter->status = 2;
        return sliding_window_filter->status;
    }

    sliding_window_filter->data_flow = (float *)malloc(data_depth * sizeof(float));

    for (uint8_t i = 0; i < data_depth; i++)
    {
        sliding_window_filter->data_flow[i] = 0.0f;
    }

    sliding_window_filter->status = 0;
    return sliding_window_filter->status;
}

/**
 * @name rlfSlidingWindowFilterCalc
 * @brief 滑动窗口滤波计算
 * @param sliding_window_filter 滑动窗口滤波结构体
 * @param input 输入的数据
 * @return float
 */
float rlfSlidingWindowFilterCalc(sliding_window_filter_s_t *sliding_window_filter, float input)
{
    // 缓存、和、窗数据数量
    float temp = 0.0f, sum = 0.0f;

    // 若滤波器结构有误，不进行计算
    if (sliding_window_filter->status)
    {
        sliding_window_filter->output = 0.0f;
        return sliding_window_filter->output;
    }

    // 原始数据滑动一位
    for (uint8_t i = 0; i < sliding_window_filter->data_depth - 1; i++)
    {
        sliding_window_filter->data_flow[i] = sliding_window_filter->data_flow[i + 1];
    }
    sliding_window_filter->data_flow[sliding_window_filter->data_depth - 1] = input;

    // 初始阶段滤波器数据未满，跳过本次计算
    if (fabsf(sliding_window_filter->data_flow[0]) < 0.0001f)
    {
        sliding_window_filter->output = 0.0f;
        return sliding_window_filter->output;
    }

    // 创建本次运算所需临时数据空间
    float *data_flow = (float *)malloc(sliding_window_filter->data_depth * sizeof(float));

    // 复制原始数据到临时数据空间，以便在不改变原始数据顺序的情况下进行排序、计算均值等
    memcpy(data_flow, sliding_window_filter->data_flow, sliding_window_filter->data_depth * sizeof(float));
    // for (uint8_t i = 0; i < sliding_window_filter->data_depth; i++)
    // {
    //     data_flow[i] = sliding_window_filter->data_flow[i];
    // }

    // 冒泡排序
    for (uint8_t i = 0; i < (sliding_window_filter->data_depth - 1); i++)
    {
        for (uint8_t j = 0; j < (sliding_window_filter->data_depth - 1 - i); j++)
        {
            if (data_flow[j] > data_flow[j + 1])
            {
                temp = data_flow[j];
                data_flow[j] = data_flow[j + 1];
                data_flow[j + 1] = temp;
            }
        }
    }

    // 两两平均求和
    for (uint8_t i = sliding_window_filter->wall_depth + 1;
         i < (sliding_window_filter->wall_depth + sliding_window_filter->window_depth); i++)
    {
        sum += ((data_flow[i] + data_flow[i - 1]) / 2.0f);
    }

    // 求均值
    sliding_window_filter->output = sum / ((float)sliding_window_filter->window_depth - 1.0f);

    // 释放临时数据空间
    free(data_flow);

    // 返回输出
    return sliding_window_filter->output;
}
