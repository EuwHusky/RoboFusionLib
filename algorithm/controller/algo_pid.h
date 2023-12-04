#ifndef _ALGO_PID_H__
#define _ALGO_PID_H__

#include "stddef.h"
#include "stdint.h"

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    // PID 三参数
    float Kp;
    float Ki;
    float Kd;

    float max_out;  // 最大输出
    float max_iout; // 最大积分输出

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  // 微分项 0最新 1上一次 2上上次
    float error[3]; // 误差项 0最新 1上一次 2上上次

} pid_type_def;

/**
 * @brief          pid struct data init
 * @param[out]     pid: PID结构数据指针
 * @param[in]      mode: PID_POSITION:普通PID
 *                 PID_DELTA: 差分PID
 * @param[in]      PID: 0: kp, 1: ki, 2:kd
 * @param[in]      max_out: pid最大输出
 * @param[in]      max_iout: pid最大积分输出
 * @retval         none
 */
extern void PID_init(pid_type_def *pid, uint8_t mode, const float PID[3], float max_out, float max_iout);

/**
 * @brief          pid计算
 * @param[out]     pid: PID结构数据指针
 * @param[in]      ref: 反馈数据
 * @param[in]      set: 设定值
 * @retval         pid输出
 */
extern float PID_calc(pid_type_def *pid, float ref, float set);

/**
 * @brief          pid 输出清除
 * @param[out]     pid: PID结构数据指针
 * @retval         none
 */
extern void PID_clear(pid_type_def *pid);

typedef struct
{
    uint8_t mode;

    // 二次P 参数
    float scale_factor; // 放缩系数
    float p_base;       // 基准值

    // PID 三参数
    float Kp;
    float Ki;
    float Kd;

    float max_out;  // 最大输出
    float max_iout; // 最大积分输出

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  // 微分项 0最新 1上一次 2上上次
    float error[3]; // 误差项 0最新 1上一次 2上上次

} Quadratic_pid_type_def;

extern void Quadratic_PID_init(Quadratic_pid_type_def *pid, uint8_t mode, const float PID[3], const float value[2],
                               float max_out, float max_iout);

extern float Quadratic_PID_calc(Quadratic_pid_type_def *pid, float ref, float set);

extern void Quadratic_PID_clear(Quadratic_pid_type_def *pid);

#endif /* _ALGO_PID_H__ */
