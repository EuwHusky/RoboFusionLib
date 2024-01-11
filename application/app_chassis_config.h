#ifndef _APP_CHASSIS_CONFIG_H__
#define _APP_CHASSIS_CONFIG_H__

#include "stdint.h"

typedef enum RflChassisType
{
    RFL_CHASSIS_MECANUM,         // 麦克纳姆底盘
    RFL_CHASSIS_COAXIAL_MECANUM, // 共轴麦克纳姆底盘
    RFL_CHASSIS_FOUR_STEER,      // 四舵底盘
    RFL_CHASSIS_DUAL_STEER,      // 双舵底盘
    RFL_CHASSIS_TYPE_NUM
} rfl_chassis_type_e;

typedef enum RflChassisControlMode
{
    RFL_CHASSIS_CONTROL_MODE_, //
    RFL_CHASSIS_TYPE_NUM
} rfl_chassis_control_mode_e;

typedef struct RflAppChassisConfig
{
    rfl_chassis_type_e type;
    rfl_chassis_control_mode_e mode;
} rfl_chassis_config_s;

#endif /* _APP_CHASSIS_CONFIG_H__ */
