#ifndef _DEV_BASE_MOTOR_H__
#define _DEV_BASE_MOTOR_H__

#include "dev_base_motor_config.h"

#include "algo_result.h"

typedef struct
{
    RflMotorType type_;
    RflMotorControlMode mode_;
    RflMotorControlMode last_mode_;
    RflMotorAngleFormat angle_format_;
    bool is_reversed_;
    float position_conversion_factor_;
    float control_period_factor_;

    RflMotorControllerType controller_type_;
    void *controller_;

    float set_speed_;
    float max_speed_;
    float set_position_;
    float track_position_;
    float max_position_;
    float min_position_;

    float speed_;
    float internal_speed_;
    float *external_speed_;
    float position_;
    float internal_position_;
    float *external_position_;
    float torque_;
    float temperature_;

    RflResult (*UpdateState)(void *);
    RflResult (*UpdateControl)(void *);
    RflResult (*ResetPosition)(void *);
    RflResult (*SetMode)(void *, RflMotorControlMode);
    RflResult (*SetSpeed)(void *, float);
    RflResult (*SetMaxSpeed)(void *, float);
    RflResult (*SetPosition)(void *, float);
    RflResult (*SetPositionLimit)(void *, float, float);
    RflResult (*GetMode)(void *);
    RflResult (*GetSpeed)(void *);
    RflResult (*GetInternalSpeed)(void *);
    RflResult (*GetMaxSpeed)(void *);
    RflResult (*GetPosition)(void *);
    RflResult (*GetInternalPosition)(void *);
    RflResult (*GetMaxPosition)(void *);
    RflResult (*GetMinPosition)(void *);
    RflResult (*GetTemperature)(void *);
    RflResult (*GetTorque)(void *);

} RflBaseMotor;

extern void RflBaseMotorGetDefaultConfig(RflBaseMotorConfig *config, RflMotorType type);
extern void RflBaseMotorInit(RflBaseMotor *motor, RflBaseMotorConfig *config);

#endif /* _DEV_BASE_MOTOR_H__ */
