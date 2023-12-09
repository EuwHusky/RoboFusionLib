#ifndef _DEV_MOTOR_CONTROLLER__
#define _DEV_MOTOR_CONTROLLER__

#include "dev_motor_config.h"

#include "algo_pid.h"

typedef struct RflMotorPidController
{
    pid_type_def speed_pid; // 速度控制PID控制器
    pid_type_def angle_pid; // 角度控制PID控制器
} rfl_motor_pid_controller_s;

#endif /* _DEV_MOTOR_CONTROLLER__ */
