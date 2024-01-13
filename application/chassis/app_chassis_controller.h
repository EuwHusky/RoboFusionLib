#ifndef _APP_CHASSIS_CONTROLLER__
#define _APP_CHASSIS_CONTROLLER__

#include "dev_motor_config.h"

#include "algo_pid.h"

typedef struct RflChassisNormalPidController
{
    pid_type_def angle_pid; // 底盘角度控制PID控制器
} rfl_chassis_normal_pid_controller_s;

typedef struct RflChassisFeedforwardPidController
{
    uint8_t struct_keep;
} rfl_chassis_feedforward_pid_controller_s;

#endif /* _APP_CHASSIS_CONTROLLER__ */
