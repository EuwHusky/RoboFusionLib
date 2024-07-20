#ifndef _DEV_MOTOR__
#define _DEV_MOTOR__

#include "dev_motor_config.h"
#include "dev_motor_controller.h"

#include "algo_angle.h"

/**
 * @brief 电机
 */
typedef struct RflMotor
{
    /* 基础参数 */

    rfl_motor_type_e type;
    rfl_motor_controller_type_e controller_type;
    rfl_motor_control_mode_e mode_;
    rfl_motor_control_mode_e last_mode;

    rfl_motor_angle_format_e angle_format; // 角度格式
    rfl_angle_s max_angle_;                // 最大控制角度
    rfl_angle_s min_angle_;                // 最小控制角度

    float control_period_factor; // 控制周期系数（由于控制周期不确定和PID控制的滞后性故仅以系数提供） 量纲 时间

    bool is_reversed; // 是否反转 用于适应电机安装极性

    /* 控制量 */

    float set_speed_; // 预期速度 单位 rad * s^-1
    float max_speed_; // 最大速度（仅作用于速度角度模式） 量纲 角度值/时间

    rfl_angle_s set_angle_;  // 预期角度
    rfl_angle_s track_angle; // 规划跟踪角度

    void *controller; // 电机控制器

    float control_output_; // 输出控制量 物理意义视用法而定

    /* 状态量 */

    float torque_; // 电机转矩 单位 N * M

    float speed_; // 末端执行器转速 单位 rad * s^-1

    rfl_angle_s angle_;         // 末端执行器角度 逆时针为正
    rfl_angle_s internal_angle; // 末端执行器角度 逆时针为正 数据源为电机自身反馈

    float temperature_; // 电机温度 单位 °C

    const float *external_speed;       // 外部速度 逆时针为正 单位 rad * s^-1
    const rfl_angle_s *external_angle; // 外部角度 逆时针为正

    /* 电机驱动 */
    void *driver;

    /* 错误码 */
    uint32_t error_code;

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
extern void rflMotorUpdateControl(rfl_motor_s *motor);
/**
 * @brief 电机执行控制
 */
extern void rflMotorExecuteControl(rfl_motor_s *motor);

/**
 * @brief 重置电机角度
 */
extern void rflMotorResetAngle(rfl_motor_s *motor, rfl_angle_format_e angle_format, float source_angle,
                               bool security_restriction);

/**
 * @brief 设置电机控制模式
 */
extern void rflMotorSetMode(rfl_motor_s *motor, rfl_motor_control_mode_e mode);
/**
 * @brief 设置电机预期角速度
 */
extern void rflMotorSetSpeed(rfl_motor_s *motor, float set_speed);
/**
 * @brief 设置电机最大角速度
 */
extern void rflMotorSetMaxSpeed(rfl_motor_s *motor, float max_speed);
/**
 * @brief 设置电机预期角度
 */
extern void rflMotorSetAngle(rfl_motor_s *motor, rfl_angle_format_e angle_format, float angle);
/**
 * @brief 设置电机角度范围-角度值
 */
extern void rflMotorSetDegAngleLimit(rfl_motor_s *motor, rfl_angle_format_e angle_format, float max_angle,
                                     float min_angle);

/**
 * @brief 获取电机当前模式
 */
extern rfl_motor_control_mode_e rflMotorGetMode(rfl_motor_s *motor);
/**
 * @brief 获取电机当前转矩
 */
extern float rflMotorGetTorque(rfl_motor_s *motor);
/**
 * @brief 获取电机最大可达角度
 */
extern float rflMotorGetMaxAngle(rfl_motor_s *motor, rfl_angle_format_e angle_format);
/**
 * @brief 获取电机最小可达角度
 */
extern float rflMotorGetMinAngle(rfl_motor_s *motor, rfl_angle_format_e angle_format);
/**
 * @brief 获取电机当前速度 单位 rad * s^-1
 */
extern float rflMotorGetSpeed(rfl_motor_s *motor);
/**
 * @brief 获取电机当前角度
 */
extern float rflMotorGetAngle(rfl_motor_s *motor, rfl_angle_format_e angle_format);
/**
 * @brief 获取电机当前自反馈角度
 */
extern float rflMotorGetInternalAngle(rfl_motor_s *motor, rfl_angle_format_e angle_format);
/**
 * @brief 获取电机当前温度
 */
extern float rflMotorGetTemperature(rfl_motor_s *motor);
/**
 * @brief 获取电机当前输出
 */
extern float rflMotorGetOutput(rfl_motor_s *motor);

#endif /* _DEV_MOTOR__ */
