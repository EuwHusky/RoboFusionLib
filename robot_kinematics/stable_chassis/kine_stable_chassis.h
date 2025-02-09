/**
 * @file kine_stable_chassis.h
 * @author EHusky
 * @brief 机器人运动学模块库底盘模块
 *
 * @copyright Copyright (c) 2024 CNU W.PIE
 *
 */

#ifndef _KINE_STABLE_CHASSIS_H__
#define _KINE_STABLE_CHASSIS_H__

#include "stdbool.h"

#include "kine_stable_chassis_config.h"

/**
 * @brief 底盘
 * @note 电机顺序从0开始 假设底盘向其正方向有一射线 并向着逆时针方向扫掠 扫掠到的电机次序作为电机顺序
 *       若是舵轮底盘则优先记录所有舵向电机顺序后 再重复一遍上述过程记录驱动电机顺序
 */
typedef struct RflChassis
{
    rfl_chassis_type_e type;      /*底盘类型*/
    rfl_chassis_behavior_e mode_; /*底盘行为模式*/
    void *parameter;              /*底盘特有参数*/

    rfl_chassis_frame_e reference_frame; /*底盘结构参考系*/
    /*参考坐标系下底盘结构方向 以结构的正面朝向为X轴正方向 向上为Z轴正方向 范围 -π ~ π 以此可以确定底盘结构坐标系*/
    const rfl_angle_s *forward_vector_;
    /* 在底盘控制坐标系下的底盘速度向量
     * 0 - 底盘前进后退速度 单位-m/s
     * 1 - 底盘左移右移速度 单位-m/s
     * 2 - 底盘旋转角速度 逆时针为正 单位-rad/s */
    float speed_vector_[3];

    /*参考坐标系下底盘控制方向 以控制时移动的前方为X轴正方向 向上为Z轴正方向 范围 -π ~ π 以此可以确定底盘控制坐标系*/
    rfl_angle_s *control_vector;
    /*设定的参考坐标系下底盘结构方向 以结构的正面朝向为X轴正方向 向上为Z轴正方向 范围 -π ~ π*/
    rfl_angle_s set_forward_vector;
    float set_vx_; /*设定的在底盘控制坐标系下的底盘前进后退速度 由用户设定 单位-m/s*/
    float set_vy_; /*设定的在底盘控制坐标系下的底盘左移右移速度 由用户设定 单位-m/s*/
    float set_wz_; /*设定的在底盘控制坐标系下的底盘旋转角速度 由底盘方位控制器设定 逆时针为正 单位-rad/s*/
    void *direction_controller; /*底盘方向控制器*/

    uint8_t motor_num; /*底盘电机数量*/
    /* 底盘电机组反馈量 数组大小为电机数量 数组下标为电机顺序 数组元素为单个电机的反馈值
     * 对于驱动电机此项为速度 向前为正 单位-m/s 对于舵向电机此项为角度 逆时针为正 单位-degree
     * 当底盘配置为不内置电机时需用户传入此项的指针*/
    const float *motor_feedback;
    /* 底盘电机组控制量 数组大小为电机数量 数组下标为电机顺序 数组元素为单个电机的反馈值
     * 对于驱动电机此项为速度 向前为正 单位-m/s 对于舵向电机此项为角度 逆时针为正 单位-degree
     * 当底盘配置为不内置电机时需用户传入此项的指针*/
    float *motor_output_;
    bool *no_force_the_motor_;                        /*无力电机*/
    bool *reverse_steer_chassis_driving_motor_output; /*反转舵轮底盘驱动电机输出方向*/
} rfl_chassis_s;

/**
 * @brief 获取底盘默认配置
 *
 * @param config 底盘配置结构体指针
 * @param type 底盘类型
 */
extern void rflChassisGetDefaultConfig(rfl_chassis_config_s *config, rfl_chassis_type_e type);

/**
 * @brief 初始化底盘实体
 *
 * @param chassis 底盘实体结构体指针
 * @param config 底盘配置结构体指针
 * @param forward_vector 参考坐标系下底盘结构方向
 * @param motor_feedback 底盘电机组反馈量
 */
extern void rflChassisInit(rfl_chassis_s *chassis, rfl_chassis_config_s *config, const rfl_angle_s *forward_vector,
                           const float *motor_feedback);

/**
 * @brief 更新底盘实体
 *
 * @param chassis 底盘实体结构体指针
 */
extern void rflChassisUpdate(rfl_chassis_s *chassis);

/**
 * @brief 设定底盘速度
 *
 * @param chassis 底盘实体结构体指针
 * @param vx 要设定的前进后退速度
 * @param vy 要设定的左移右移速度
 * @param wz 要设定的小陀螺自转速度
 */
extern void rflChassisSetSpeedVector(rfl_chassis_s *chassis, float vx, float vy, float wz);

/**
 * @brief 设定底盘行为模式
 *
 * @param chassis 底盘实体结构体指针
 * @param mode 要设定的行为模式
 */
extern void rflChassisSetBehavior(rfl_chassis_s *chassis, rfl_chassis_behavior_e mode);

/**
 * @brief 获取底盘速度向量
 *
 * @param chassis 底盘实体结构体指针
 * @return float* 在底盘控制坐标系下的底盘速度向量
 */
extern float *rflChassisGetSpeedVector(rfl_chassis_s *chassis);

/**
 * @brief 获取底盘电机组输出值
 *
 * @param chassis 底盘实体结构体指针
 * @return float* 底盘电机组输出值
 */
extern float *rflChassisGetMotorOutputArray(rfl_chassis_s *chassis);

/**
 * @brief 获取底盘电机组当前模式
 *
 * @param chassis 底盘实体结构体指针
 * @return float* 底盘电机组模式
 */
extern bool *rflChassisGetMotorModeArray(rfl_chassis_s *chassis);

#endif /* _KINE_STABLE_CHASSIS_H__ */
