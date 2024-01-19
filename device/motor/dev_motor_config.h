#ifndef _DEV_MOTOR_CONFIG_H__
#define _DEV_MOTOR_CONFIG_H__

#include "stdbool.h"
#include "stdint.h"

#include "rfl_config.h"

#include "algo_angle.h"

/**
 * @brief 电机类型
 */
typedef enum RflMotorType
{
#if (RFL_DEV_MOTOR_RM_MOTOR == 1)
    RFL_MOTOR_RM_M2006,
    RFL_MOTOR_RM_M3508,
    RFL_MOTOR_RM_GM6020,
#endif /* RFL_DEV_MOTOR_RM_MOTOR == 1 */

#if (RFL_DEV_MOTOR_UNITREE_MOTOR == 1)
    RFL_MOTOR_UNITREE_GO_M8010_6,
#endif /* RFL_DEV_MOTOR_UNITREE_MOTOR == 1 */

    RFL_MOTOR_TYPE_NUM
} rfl_motor_type_e;

/**
 * @brief 电机控制器
 */
typedef enum RflMotorControllerType
{
    RFL_MOTOR_CONTROLLER_PID, // PID控制器

    RFL_MOTOR_CONTROLLER_UNITREE, // Unitree直接控制器
} rfl_motor_controller_type_e;

/**
 * @brief 电机控制模式
 */
typedef enum RflMotorControlMode
{
    RFL_MOTOR_CONTROL_MODE_NO_FORCE = 0, // 无力
    RFL_MOTOR_CONTROL_MODE_SPEED,        // 速度
    RFL_MOTOR_CONTROL_MODE_ANGLE,        // 角度
    RFL_MOTOR_CONTROL_MODE_DIRECTION,    // 方位
    RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE,  // 速度角度
} rfl_motor_control_mode_e;

/**
 * @brief 角度计算模式
 */
typedef enum RflMotorAngleFormat
{
    RFL_MOTOR_ANGLE_FORMAT_CIRCLED = 0, // 多圈角度 -10000° ~ 10000°
    RFL_MOTOR_ANGLE_FORMAT_ABSOLUTE     // 绝对角度 -180° ~ 180°
} rfl_motor_angle_format_e;

/**
 * @brief 电机配置
 */
typedef struct RflMotorConfig
{
    rfl_motor_type_e type;
    rfl_motor_controller_type_e controller_type;
    rfl_motor_control_mode_e mode;
    float control_period_factor;
    rfl_motor_angle_format_e angle_format;
    float effector_transmission_ratio;
    bool is_reversed;

    float max_speed;
    rfl_angle_s max_angle;
    rfl_angle_s min_angle;

    float speed_pid_kp;
    float speed_pid_ki;
    float speed_pid_kd;
    float speed_pid_max_iout;
    float speed_pid_max_out;
    float angle_pid_kp;
    float angle_pid_ki;
    float angle_pid_kd;
    float angle_pid_max_iout;
    float angle_pid_max_out;

    float unitree_k_a;
    float unitree_k_s;

    const float *external_speed;
    const rfl_angle_s *external_angle;

#if (RFL_DEV_MOTOR_RM_MOTOR == 1)
    uint8_t can_ordinal;
    uint32_t can_id;
#endif /* RFL_DEV_MOTOR_RM_MOTOR == 1 */

#if (RFL_DEV_MOTOR_UNITREE_MOTOR == 1)
    uint16_t unitree_motor_id;
#endif /* RFL_DEV_MOTOR_UNITREE_MOTOR == 1 */

} rfl_motor_config_s;

/* 电机模块通用参数 --------------------------- */

#define RFL_MOTOR_DEFAULT_CONTROL_PERIOD_FACTOR (1.0f)
#define RFL_MOTOR_DEFAULT_MAX_SPEED (1.0f)

#define RFL_MOTOR_DEFAULT_ANGLE_RANGE (360.0f)

// 默认电机PID参数
#define RFL_MOTOR_DEFAULT_SPEED_PID_KP (0.0f)
#define RFL_MOTOR_DEFAULT_SPEED_PID_KI (0.0f)
#define RFL_MOTOR_DEFAULT_SPEED_PID_KD (0.0f)
#define RFL_MOTOR_DEFAULT_SPEED_PID_MAX_IOUT (0.0f)
#define RFL_MOTOR_DEFAULT_SPEED_PID_MAX_OUT (0.0f)
#define RFL_MOTOR_DEFAULT_ANGLE_PID_KP (0.0f)
#define RFL_MOTOR_DEFAULT_ANGLE_PID_KI (0.0f)
#define RFL_MOTOR_DEFAULT_ANGLE_PID_KD (0.0f)
#define RFL_MOTOR_DEFAULT_ANGLE_PID_MAX_IOUT (0.0f)
#define RFL_MOTOR_DEFAULT_ANGLE_PID_MAX_OUT (0.0f)

#if (RFL_DEV_MOTOR_RM_MOTOR == 1)
// RM M2006 PID参数
#define RFL_MOTOR_RM_M2006_DEFAULT_ANGLE_PID_KP (1.0f)
#define RFL_MOTOR_RM_M2006_DEFAULT_ANGLE_PID_KI (0.0f)
#define RFL_MOTOR_RM_M2006_DEFAULT_ANGLE_PID_KD (0.2f)
#define RFL_MOTOR_RM_M2006_DEFAULT_ANGLE_PID_MAX_IOUT (0.0f)
#define RFL_MOTOR_RM_M2006_DEFAULT_ANGLE_PID_MAX_OUT (24.0f)
#define RFL_MOTOR_RM_M2006_DEFAULT_SPEED_PID_KP (4000.0f)
#define RFL_MOTOR_RM_M2006_DEFAULT_SPEED_PID_KI (200.0f)
#define RFL_MOTOR_RM_M2006_DEFAULT_SPEED_PID_KD (0.0f)
#define RFL_MOTOR_RM_M2006_DEFAULT_SPEED_PID_MAX_IOUT (1000.0f)
#define RFL_MOTOR_RM_M2006_DEFAULT_SPEED_PID_MAX_OUT (10000.0f)
// RM M3508 PID参数
#define RFL_MOTOR_RM_M3508_DEFAULT_ANGLE_PID_KP (0.8f)
#define RFL_MOTOR_RM_M3508_DEFAULT_ANGLE_PID_KI (0.0f)
#define RFL_MOTOR_RM_M3508_DEFAULT_ANGLE_PID_KD (0.0f)
#define RFL_MOTOR_RM_M3508_DEFAULT_ANGLE_PID_MAX_IOUT (0.0f)
#define RFL_MOTOR_RM_M3508_DEFAULT_ANGLE_PID_MAX_OUT (16.0f)
#define RFL_MOTOR_RM_M3508_DEFAULT_SPEED_PID_KP (1200.0f)
#define RFL_MOTOR_RM_M3508_DEFAULT_SPEED_PID_KI (12.0f)
#define RFL_MOTOR_RM_M3508_DEFAULT_SPEED_PID_KD (0.0f)
#define RFL_MOTOR_RM_M3508_DEFAULT_SPEED_PID_MAX_IOUT (4000.0f)
#define RFL_MOTOR_RM_M3508_DEFAULT_SPEED_PID_MAX_OUT (16000.0f)
// RM GM6020 PID参数
#define RFL_MOTOR_RM_GM6020_DEFAULT_ANGLE_PID_KP (1.0f)
#define RFL_MOTOR_RM_GM6020_DEFAULT_ANGLE_PID_KI (0.0f)
#define RFL_MOTOR_RM_GM6020_DEFAULT_ANGLE_PID_KD (0.0f)
#define RFL_MOTOR_RM_GM6020_DEFAULT_ANGLE_PID_MAX_IOUT (0.0f)
#define RFL_MOTOR_RM_GM6020_DEFAULT_ANGLE_PID_MAX_OUT (42.0f)
#define RFL_MOTOR_RM_GM6020_DEFAULT_SPEED_PID_KP (500.0f)
#define RFL_MOTOR_RM_GM6020_DEFAULT_SPEED_PID_KI (50.0f)
#define RFL_MOTOR_RM_GM6020_DEFAULT_SPEED_PID_KD (0.0f)
#define RFL_MOTOR_RM_GM6020_DEFAULT_SPEED_PID_MAX_IOUT (20000.0f)
#define RFL_MOTOR_RM_GM6020_DEFAULT_SPEED_PID_MAX_OUT (30000.0f)
#endif /* RFL_DEV_MOTOR_RM_MOTOR == 1 */

#if (RFL_DEV_MOTOR_UNITREE_MOTOR == 1)
#define RFL_MOTOR_UNITREE_GO_M8010_6_K_ANGLE (0.4f)
#define RFL_MOTOR_UNITREE_GO_M8010_6_K_SPEED (0.02f)
// #define RFL_MOTOR_UNITREE_GO_M8010_6_K_ANGLE (5.0f)
// #define RFL_MOTOR_UNITREE_GO_M8010_6_K_SPEED (0.02f)

#define RFL_MOTOR_UNITREE_DEFAULT_K_ANGLE (0.0f)
#define RFL_MOTOR_UNITREE_DEFAULT_K_SPEED (0.0f)
#endif /* RFL_DEV_MOTOR_UNITREE_MOTOR == 1 */

#endif /* _DEV_MOTOR_CONFIG_H__ */
