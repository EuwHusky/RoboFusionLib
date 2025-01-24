#ifndef _DEV_BASE_MOTOR_H__
#define _DEV_BASE_MOTOR_H__

#include "dev_base_motor_config.h"

typedef struct
{
    RflMotorType type_;
    RflMotorControlMode mode_;
    RflMotorControlMode last_mode_;
    RflMotorAngleFormat angle_format_; // 角度格式
    bool is_reversed_;                 // 反转
    float position_conversion_factor_; // 末端执行器位置转换系数
    float control_period_factor_; // 控制周期系数（由于控制周期不确定和PID控制的滞后性故仅以系数提供） 量纲 时间

    RflMotorControllerType controller_type_;
    void *controller_;

    float set_speed_;      // 设定转速
    float max_speed_;      // 最大速度
    float set_position_;   // 设定位置
    float track_position_; // 跟随位置
    float max_position_;   // 最大位置
    float min_position_;   // 最小位置

    float speed_;              // 末端执行器转速
    float internal_speed_;     // 末端执行器转速 数据源为电机自身反馈
    float *external_speed_;    // 末端执行器转速 数据源为用户输入
    float position_;           // 末端执行器位置
    float internal_position_;  // 末端执行器位置 数据源为电机自身反馈
    float *external_position_; // 末端执行器位置 数据源为用户输入
    float torque_;             // 电机转矩
    float temperature_;        // 电机温度

    void (*UpdateState)(void *);
    void (*UpdateControl)(void *);
    void (*ResetPosition)(void *);
    RflMotorError (*SetMode)(void *, RflMotorControlMode);
    void (*SetSpeed)(void *, float);
    void (*SetMaxSpeed)(void *, float);
    void (*SetPosition)(void *, float);
    void (*SetPositionLimit)(void *, float, float);
    RflMotorControlMode (*GetMode)(void *);
    float (*GetSpeed)(void *);
    float (*GetInternalSpeed)(void *);
    float (*GetMaxSpeed)(void *);
    float (*GetPosition)(void *);
    float (*GetInternalPosition)(void *);
    float (*GetMaxPosition)(void *);
    float (*GetMinPosition)(void *);
    float (*GetTemperature)(void *);
    float (*GetTorque)(void *);

} RflBaseMotor;

extern void RflBaseMotorGetDefaultConfig(RflBaseMotorConfig *config, RflMotorType type);
extern void RflBaseMotorInit(RflBaseMotor *motor, RflBaseMotorConfig *config);

#endif /* _DEV_BASE_MOTOR_H__ */
