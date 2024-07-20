/**
 * @file kine_stable_chassis_config.h
 * @author EHusky
 * @brief 机器人运动学模块库底盘模块 配置
 *
 * @copyright Copyright (c) 2024 CNU W.PIE
 *
 */

#ifndef _KINE_STABLE_CHASSIS_CONFIG_H__
#define _KINE_STABLE_CHASSIS_CONFIG_H__

#include "stdint.h"

#include "algo_angle.h"

/**
 * @brief 底盘类型
 */
typedef enum RflChassisType
{
    RFL_CHASSIS_MECANUM = 0, /*上X下O型麦克纳姆轮底盘*/
    RFL_CHASSIS_OMNI,        /*全向轮底盘*/
    RFL_CHASSIS_DUAL_STEER,  /*两舵轮底盘*/
    RFL_CHASSIS_FOUR_STEER,  /*四舵轮底盘*/
} rfl_chassis_type_e;

/**
 * @brief 底盘参考坐标系类型
 */
typedef enum RflChassisFrame
{
    RFL_CHASSIS_INERTIAL_FRAME = 0, /*惯性坐标系 既IMU偏转地理坐标系*/
    RFL_CHASSIS_CONTROL_FRAME,      /*控制坐标系*/
} rfl_chassis_frame_e;

/**
 * @brief 底盘方向控制器类型
 */
typedef enum RflChassisDirectionControllerType
{
    RFL_CHASSIS_CONTROLLER_PID,             /*PID控制器*/
    RFL_CHASSIS_CONTROLLER_FEEDFORWARD_PID, /*前馈+PID控制器*/
} rfl_chassis_direction_controller_type_e;

/**
 * @brief 底盘行为模式
 */
typedef enum RflChassisBehavior
{
    RFL_CHASSIS_BEHAVIOR_NO_FORCE = 0,   /*底盘无力*/
    RFL_CHASSIS_BEHAVIOR_FOLLOW_CONTROL, /*底盘结构正方向跟随控制正方向*/
    RFL_CHASSIS_BEHAVIOR_FREEZE,         /*底盘结构正方向固定*/
    RFL_CHASSIS_BEHAVIOR_SPIN,           /*底盘结构正方向不断变化 小陀螺*/
} rfl_chassis_behavior_e;

/**
 * @brief 底盘配置
 * @note 底盘电机顺序定义：假设底盘向其结构正方向有一射线 并向着逆时针方向扫掠 将扫掠到电机的次序定义为电机顺序
 *       若是舵轮底盘则优先记录下所有驱动电机的顺序 然后再重复上述步骤记录舵向电机的顺序
 */
typedef struct RflChassisConfig
{
    rfl_chassis_type_e type;         /*底盘类型*/
    rfl_chassis_behavior_e behavior; /*底盘行为*/
    float mecanum_width; /*上X下O型麦克纳姆轮底盘四个轮子与地面的接触点形成的矩形平行于Y方向的边长 单位-m*/
    float mecanum_length; /*上X下O型麦克纳姆轮底盘四个轮子与地面的接触点形成的矩形平行于X方向的边长*/
    float length; /*全向/舵轮底盘轮子与地面的接触点到底盘旋转中心的距离 单位-m*/
    uint8_t steer_motor_orientation; /*舵轮方位 0-舵轮位于左前右后方向 非0-舵轮位于左后右前方向*/

    rfl_chassis_frame_e reference_frame; /*底盘结构参考系*/

    /*惯性坐标系下用户设定的底盘控制方向 以控制时移动的前方为X轴正方向 向上为Z轴正方向 范围 -π ~ π
     * 以此可以确定底盘控制坐标系*/
    const rfl_angle_s *set_control_vector_;
    rfl_chassis_direction_controller_type_e direction_controller_type; /*底盘方向控制器类型*/
    float direction_pid_param[5];                                      /*底盘方向PID控制器参数*/
} rfl_chassis_config_s;

#endif /* _KINE_STABLE_CHASSIS_CONFIG_H__ */
