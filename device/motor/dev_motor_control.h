#ifndef _DEV_MOTOR_CONTROL_H__
#define _DEV_MOTOR_CONTROL_H__

#include "dev_motor.h"

extern void rfl_motor_pid_no_force_control(rfl_motor_s *motor);
extern void rfl_motor_pid_speed_control(rfl_motor_s *motor);
extern void rfl_motor_pid_angle_control(rfl_motor_s *motor);
extern void rfl_motor_pid_direction_control(rfl_motor_s *motor);
extern void rfl_motor_pid_speed_angle_control(rfl_motor_s *motor);

#if (RFL_DEV_MOTOR_UNITREE_MOTOR == 1)
extern void rfl_motor_unitree_no_force_control(rfl_motor_s *motor);
extern void rfl_motor_unitree_angle_control(rfl_motor_s *motor);
extern void rfl_motor_unitree_speed_angle_control(rfl_motor_s *motor);
#endif /* RFL_DEV_MOTOR_UNITREE_MOTOR == 1 */

#if (RFL_DEV_MOTOR_DAMIAO_MOTOR == 1)
extern void rfl_motor_damiao_no_force_control(rfl_motor_s *motor);
extern void rfl_motor_damiao_speed_angle_control(rfl_motor_s *motor);
#endif /* RFL_DEV_MOTOR_DAMIAO_MOTOR == 1 */

#endif /* _DEV_MOTOR_CONTROL_H__ */
