#ifndef _DEV_MOTOR__
#define _DEV_MOTOR__

#include "dev_motor_config.h"
#include "dev_motor_controller.h"

#include "algo_angle.h"

#if (RFL_DEV_MOTOR_RM_MOTOR == 1)
#include "bsp_rm_motor.h"
#endif /* RFL_DEV_MOTOR_RM_MOTOR == 1 */

#if (RFL_DEV_MOTOR_UNITREE_MOTOR == 1)
#include "bsp_unitree_motor.h"
#endif /* RFL_DEV_MOTOR_UNITREE_MOTOR == 1 */

/**
 * @brief 电机
 */
typedef struct RflMotor
{
    /* 基础参数 */

    rfl_motor_type_e type;
    rfl_motor_controller_type_e controller_type;
    rfl_motor_control_mode_e mode_;

    rfl_motor_angle_format_e angle_format; // 角度格式

    float effector_transmission_ratio_; // 电机末端执行器转一圈时电机转子转过的圈数

    /* 控制量 */

    float set_speed_; // 预期速度 单位rad * s^-1
    float max_accle_; // 最大加速度 单位rad * s^-2

    rfl_angle_s set_angle_; // 预期角度
    rfl_angle_s max_angle_; // 最大控制角度 单位degree
    rfl_angle_s min_angle_; // 最小控制角度 单位degree

    void *controller; // 电机控制器

    float control_output; // 输出控制量 物理意义视用法而定

    /* 状态量 */

    float speed; // 末端执行器转速 单位rad * s^-1

    rfl_angle_s angle; // 末端执行器角度

    const float *external_speed;       // 外部速度
    const rfl_angle_s *external_angle; // 外部角度

    /* 电机驱动 */
    void *driver;

    /* 错误码 */
    uint8_t error_code;

} rfl_motor_s;

/**
 * @brief 获取电机默认配置
 */
extern void rflMotorGetDefaultConfig(rfl_motor_config_s *motor_config, rfl_motor_type_e type,
                                     rfl_motor_controller_type_e controller);
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
 * @brief 电机执行控制
 */
extern void rflMotorExecuteControl(rfl_motor_s *motor);

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
 * @brief 设置电机预期角度-角度值
 */
extern void rflMotorSetDegAngle(rfl_motor_s *motor, float degree_angle);
/**
 * @brief 设置电机预期角度-弧度制
 */
extern void rflMotorSetRadAngle(rfl_motor_s *motor, float radian_angle);
/**
 * @brief 设置电机角度范围-角度值
 */
extern void rflMotorSetDegAngleLimit(rfl_motor_s *motor, float max_degree_angle, float min_degree_angle);
/**
 * @brief 设置电机角度范围-弧度制
 */
extern void rflMotorSetRadAngleLimit(rfl_motor_s *motor, float max_radian_angle, float min_radian_angle);

/**
 * @brief 重置电机零位，将当前位置设为零位
 */
extern void rflMotorResetAngle(rfl_motor_s *motor);

#endif /* _DEV_MOTOR__ */
