/**
 * @file kine_stable_chassis_controller.h
 * @author EHusky
 * @brief 机器人运动学模块库底盘模块 控制器
 *
 * @copyright Copyright (c) 2024 CNU W.PIE
 *
 */

#ifndef _KINE_STABLE_CHASSIS_CONTROLLER_H__
#define _KINE_STABLE_CHASSIS_CONTROLLER_H__

#include "dev_motor_config.h"

#include "algo_pid.h"

/**
 * @brief 底盘常规PID控制器
 */
typedef struct RflChassisNormalPidController
{
    pid_type_def angle_pid; /*底盘角度PID控制器*/
} rfl_chassis_normal_pid_controller_s;

/**
 * @brief 底盘前馈+PID控制器
 */
typedef struct RflChassisFeedforwardPidController
{
    uint8_t struct_keep;
} rfl_chassis_feedforward_pid_controller_s;

/*底盘默认方向PID控制器参数*/
#define RFL_CHASSIS_DEFAULT_DIRECTION_PID_KP (0.1f)
#define RFL_CHASSIS_DEFAULT_DIRECTION_PID_KI (0.01f)
#define RFL_CHASSIS_DEFAULT_DIRECTION_PID_KD (0.0f)
#define RFL_CHASSIS_DEFAULT_DIRECTION_PID_MAX_IOUT (0.4f)
#define RFL_CHASSIS_DEFAULT_DIRECTION_PID_MAX_OUT (4.0f)

#endif /* _KINE_STABLE_CHASSIS_CONTROLLER_H__ */
