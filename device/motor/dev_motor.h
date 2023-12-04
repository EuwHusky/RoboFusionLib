#ifndef _DEV_MOTOR__
#define _DEV_MOTOR__

#include "stdint.h"

#include "dev_motor_config.h"

#include "algo_angle.h"
#include "algo_pid.h"

#if (RFL_DEV_MOTOR_RM_MOTOR == 1)
#include "bsp_rm_motor.h"
#endif /* RFL_DEV_MOTOR_RM_MOTOR == 1 */

#if (RFL_DEV_MOTOR_UNITREE_MOTOR == 1)
#include "bsp_unitree_motor.h"
#endif /* RFL_DEV_MOTOR_UNITREE_MOTOR == 1 */

/**
 * @brief 电机类型
 */
typedef enum RflMotorType
{

/* 使用RM官方电机 */
#if (RFL_DEV_MOTOR_RM_MOTOR == 1)
    RFL_MOTOR_RM_M2006 = 0,
    RFL_MOTOR_RM_M3508,
    RFL_MOTOR_RM_GM6020,
#endif /* RFL_DEV_MOTOR_RM_MOTOR == 1 */

#if (RFL_DEV_MOTOR_UNITREE_MOTOR == 1)
    RFL_MOTOR_UNITREE_GO_M8010_6,
#endif /* RFL_DEV_MOTOR_UNITREE_MOTOR == 1 */

    RFL_MOTOR_TYPE_NUM
} rfl_motor_type_e;

/**
 * @brief 电机控制模式
 */
typedef enum RflMotorControlMode
{
    RFL_MOTOR_CONTROL_MODE_NO_FORCE = 0, // 无力
    RFL_MOTOR_CONTROL_MODE_SPEED,        // 速度
    RFL_MOTOR_CONTROL_MODE_ANGLE,        // 角度
    RFL_MOTOR_CONTROL_MODE_DIRECTION     // 方位
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
 * @brief 电机
 */
typedef struct RflMotor
{
    /* 基础参数 */

    rfl_motor_type_e type;
    rfl_motor_control_mode_e mode_;

    rfl_motor_angle_format_e angle_format; // 角度格式

    float effector_transmission_ratio_; // 末端执行器转一圈时电机转子转过的圈数

    /* 控制量 */

    float set_speed_; // 预期速度 单位rad * s^-1
    float max_accle_; // 最大加速度 单位rad * s^-2

    rfl_angle_s set_angle_; // 预期角度
    rfl_angle_s max_angle_; // 最大控制角度 单位degree
    rfl_angle_s min_angle_; // 最小控制角度 单位degree

    pid_type_def speed_pid; // 速度控制PID控制器
    pid_type_def angle_pid; // 角度控制PID控制器

    float control_output; // 控制输出 无绝对物理意义

    /* 状态量 */

    float speed; // 末端执行器转速 单位rad * s^-1

    rfl_angle_s angle; // 末端执行器角度

    const float *external_speed;       // 外部速度
    const rfl_angle_s *external_angle; // 外部角度

    /* 电机驱动 */
    void *motor_driver;

    /* 错误码 */
    uint8_t error_code;

} rfl_motor_s;

/**
 * @brief 电机配置
 */
typedef struct RflMotorConfig
{
    rfl_motor_type_e type;
    rfl_motor_control_mode_e mode;
    rfl_motor_angle_format_e angle_format;
    float effector_transmission_ratio;

    float max_accle;

    rfl_angle_s max_angle;
    rfl_angle_s min_angle;

    float angle_pid_kp;
    float angle_pid_ki;
    float angle_pid_kd;
    float angle_pid_max_iout;
    float angle_pid_max_out;

    float speed_pid_kp;
    float speed_pid_ki;
    float speed_pid_kd;
    float speed_pid_max_iout;
    float speed_pid_max_out;

    const float *external_speed;
    const rfl_angle_s *external_angle;

/* 使用RM官方电机 */
#if (RFL_DEV_MOTOR_RM_MOTOR == 1)
    uint8_t can_handle_id;
    uint32_t can_id;
#endif /* RFL_DEV_MOTOR_RM_MOTOR == 1 */

} rfl_motor_config_s;

/**
 * @brief 获取电机默认配置
 */
extern void rflMotorGetDefaultConfig(rfl_motor_config_s *motor_config, rfl_motor_type_e type);
/**
 * @brief 初始化电机
 */
extern void rflMotorInit(rfl_motor_s *motor, rfl_motor_config_s *motor_config);

/**
 * @brief 更新电机状态量
 */
extern void rflMotorUpdateStatus(rfl_motor_s *motor);
/**
 * @brief 更新电机控制量
 */
extern float rflMotorUpdataControl(rfl_motor_s *motor);

/**
 * @brief 获取电机当前速度
 */
extern float rflMotorGetSpeed(rfl_motor_s *motor);
/**
 * @brief 获取电机当前角度
 */
extern rfl_angle_s *rflMotorGetAngle(rfl_motor_s *motor);
/**
 * @brief 获取电机当前输出
 */
extern float rflMotorGetOutput(rfl_motor_s *motor);

/**
 * @brief 设置电机控制模式
 */
extern void rflMotorSetMode(rfl_motor_s *motor, rfl_motor_control_mode_e mode);
/**
 * @brief 设置电机预期角速度
 */
extern void rflMotorSetSpeed(rfl_motor_s *motor, float set_speed);
/**
 * @brief 设置电机预期角加速度
 */
extern void rflMotorSetAccle(rfl_motor_s *motor, float set_accle);
/**
 * @brief 设置电机预期角度
 */
extern void rflMotorSetAngle(rfl_motor_s *motor, rfl_angle_s set_angle);
/**
 * @brief 设置电机角度范围
 */
extern void rflMotorSetAngleLimit(rfl_motor_s *motor, rfl_angle_s max_angle, rfl_angle_s min_angle);

/**
 * @brief 重置电机零位，将当前位置设为零位
 */
extern void rflMotorResetAngle(rfl_motor_s *motor);

#endif /* _DEV_MOTOR__ */
