/**
 * @file kine_stable_chassis_parameter.h
 * @author EHusky
 * @brief 机器人运动学模块库底盘模块 参数
 *
 * @copyright Copyright (c) 2024 CNU W.PIE
 *
 */

#ifndef _KINE_STABLE_CHASSIS_PARAMETER_H__
#define _KINE_STABLE_CHASSIS_PARAMETER_H__

#include "stdint.h"

/**
 * @brief 上X下O型麦克纳姆轮底盘参数
 */
typedef struct RflChassisMecanumParameter
{
    float length; /*底盘长度 定义为四个轮子与地面的接触点形成的矩形平行于X方向的边长 单位-m*/
    float width; /*底盘宽度 四个轮子与地面的接触点形成的矩形平行于Y方向的边长 单位-m*/
} rfl_chassis_mecanum_parameter_s;

/**
 * @brief 全向轮底盘参数
 */
typedef struct RflChassisOmniParameter
{
    float length; /*轮心距 定义为轮子与地面的接触点到底盘旋转中心的距离 单位-m*/
} rfl_chassis_omni_parameter_s;

/**
 * @brief 双舵轮底盘参数
 */
typedef struct RflChassisDualSteerParameter
{
    uint8_t steer_motor_orientation; /*舵向电机方位 0-舵轮位于左前右后方向 非0-舵轮位于左后右前方向*/
    float length; /*轮心距 定义为轮子与地面的接触点到底盘旋转中心的距离 单位-m*/
} rfl_chassis_dual_steer_parameter_s;

/**
 * @brief 四舵轮底盘参数
 */
typedef struct RflChassisFourSteerParameter
{
    float length; /*轮心距 定义为轮子与地面的接触点到底盘旋转中心的距离 单位-m*/
} rfl_chassis_four_steer_parameter_s;

/*上X下O型麦克纳姆轮底盘默认长度 单位-m*/
#define RFL_CHASSIS_MECANUM_DEFAULT_LENGTH (0.56f)
/*上X下O型麦克纳姆轮底盘默认宽度 单位-m*/
#define RFL_CHASSIS_MECANUM_DEFAULT_WIDTH (0.56f)
/*全向轮底盘默认轮心距 单位-m*/
#define RFL_CHASSIS_OMNI_DEFAULT_LENGTH (0.56f)
/*舵轮底盘默认轮心距 单位-m*/
#define RFL_CHASSIS_STEER_DEFAULT_LENGTH (0.56f)

#endif /* _KINE_STABLE_CHASSIS_PARAMETER_H__ */
