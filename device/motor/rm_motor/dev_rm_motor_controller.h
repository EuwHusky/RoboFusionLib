#ifndef _DEV_RM_MOTOR_CONTROLLER__
#define _DEV_RM_MOTOR_CONTROLLER__

#include "algo_pid.h"

typedef struct
{
    float speed_pid_kp;
    float speed_pid_ki;
    float speed_pid_kd;
    float speed_pid_max_iout;
    float speed_pid_max_out;
    float position_pid_kp;
    float position_pid_ki;
    float position_pid_kd;
    float position_pid_max_iout;
    float position_pid_max_out;

} RflRmMotorControllerNormalPidParam;

typedef struct
{
    pid_type_def speed_pid;
    pid_type_def position_pid;

} RflRmMotorControllerNormalPid;

extern void RflRmMotorControlValueReset(RflRmMotor *self);
extern void RflRmMotorControllerReset(RflRmMotor *self);

extern void RflRmMotorNormalPidReset(RflRmMotor *self);
extern void RflRmMotorNormalPidNoForceControl(RflRmMotor *self);
extern void RflRmMotorNormalPidSpeedControl(RflRmMotor *self);
extern void RflRmMotorNormalPidCircledPositionControl(RflRmMotor *self);
extern void RflRmMotorNormalPidAbsolutePositionControl(RflRmMotor *self);

#endif /* _DEV_RM_MOTOR_CONTROLLER__ */
