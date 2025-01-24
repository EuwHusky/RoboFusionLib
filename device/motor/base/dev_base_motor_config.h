#ifndef _DEV_BASE_MOTOR_CONFIG_H__
#define _DEV_BASE_MOTOR_CONFIG_H__

#include "stdbool.h"
#include "stdint.h"

#include "Rfl_config.h"

#include "algo_angle.h"

#define RFL_MOTOR_DEFAULT_CONTROL_PERIOD_FACTOR (1.0f)
#define RFL_MOTOR_DEFAULT_MAX_SPEED (1.0f)

#define RFL_MOTOR_DEFAULT_POSITION_RANGE (RAD_PI)

typedef enum
{
    RFL_MOTOR_SUCCESS = 0,
    RFL_MOTOR_NULL_POINTER,
    RFL_MOTOR_TYPE_MISMATCH,
    RFL_MOTOR_CONTROLLER_TYPE_MISMATCH,
    RFL_MOTOR_OUT_OF_MEMORY,
    RFL_MOTOR_RESET_POSITION_INVALID,
    RFL_MOTOR_SET_MODE_INVALID,

} RflMotorError;

typedef enum
{
    RFL_MOTOR_UNDEFINED = 0,

#if RFL_DEV_RM_MOTOR_ENABLED
    RFL_MOTOR_RM_M2006,
    RFL_MOTOR_RM_M3508,
    RFL_MOTOR_RM_GM6020,
#endif

#if RFL_DEV_UNITREE_MOTOR_ENABLED
    RFL_MOTOR_UNITREE_GO_M8010_6,
#endif

#if RFL_DEV_DAMIAO_MOTOR_ENABLED
    RFL_MOTOR_DM_J8009_2EC,
#endif

} RflMotorType;

typedef enum
{
    RFL_MOTOR_CONTROL_MODE_NO_FORCE = 0,
    RFL_MOTOR_CONTROL_MODE_SPEED,
    RFL_MOTOR_CONTROL_MODE_ANGLE,
    RFL_MOTOR_CONTROL_MODE_MIT,
} RflMotorControlMode;

typedef enum
{
    RFL_MOTOR_ANGLE_FORMAT_CIRCLED = 0,
    RFL_MOTOR_ANGLE_FORMAT_ABSOLUTE
} RflMotorAngleFormat;

typedef enum
{
    RFL_MOTOR_CONTROLLER_UNDEFINED = 0,

    RFL_RM_MOTOR_CONTROLLER_NORMAL_PID,

    RFL_UNITREE_MOTOR_CONTROLLER,

    RFL_DAMIAO_MOTOR_CONTROLLER_MIT,
    RFL_DAMIAO_MOTOR_CONTROLLER_POS_SPEED,
    RFL_DAMIAO_MOTOR_CONTROLLER_SPEED,
} RflMotorControllerType;

typedef struct
{
    RflMotorType type;
    RflMotorAngleFormat angle_format;
    bool is_reversed;
    float position_conversion_factor;
    float control_period_factor;

    RflMotorControllerType controller_type;

    float max_position;
    float min_position;

    float *external_speed;
    float *external_position;

} RflBaseMotorConfig;

#endif /* _DEV_BASE_MOTOR_CONFIG_H__ */
