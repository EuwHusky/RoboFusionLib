#ifndef _DEV_MOTOR_CONTROL_H__
#define _DEV_MOTOR_CONTROL_H__

#include "dev_motor.h"

extern void rfl_motor_no_force_control(rfl_motor_s *motor);
extern void rfl_motor_speed_control(rfl_motor_s *motor);
extern void rfl_motor_angle_control(rfl_motor_s *motor);
extern void rfl_motor_direction_control(rfl_motor_s *motor);
extern void rfl_motor_speed_angle_control(rfl_motor_s *motor);

#endif /* _DEV_MOTOR_CONTROL_H__ */
