#include "dev_motor_control.h"

#include "algo_data_limiting.h"

void rfl_motor_no_force_control(rfl_motor_s *motor)
{
    if (motor->controller_type == RFL_MOTOR_CONTROLLER_PID)
    {
        motor->control_output_ = motor->set_speed_ = motor->max_speed_ = 0.0f;
        rflAngleUpdate(&motor->track_angle, RFL_ANGLE_FORMAT_DEGREE, motor->angle_.deg);
        rflAngleUpdate(&motor->set_angle_, RFL_ANGLE_FORMAT_DEGREE, motor->angle_.deg);

        PID_clear(&((rfl_motor_pid_controller_s *)(motor->controller))->angle_pid);
        PID_clear(&((rfl_motor_pid_controller_s *)(motor->controller))->speed_pid);
    }
    else if (motor->controller_type == RFL_MOTOR_CONTROLLER_UNITREE)
    {
        motor->control_output_ = motor->set_speed_ = motor->max_speed_ = 0.0f;
        rflAngleUpdate(&motor->track_angle, RFL_ANGLE_FORMAT_DEGREE, motor->angle_.deg);
        rflAngleUpdate(&motor->set_angle_, RFL_ANGLE_FORMAT_DEGREE, motor->angle_.deg);

        ((unitree_motor_s *)(motor->driver))->set_shaft_angle = motor->set_angle_.rad;

        ((unitree_motor_s *)(motor->driver))->k_a = 0.0f;
        ((unitree_motor_s *)(motor->driver))->k_s = 0.0f;

        ((unitree_motor_s *)(motor->driver))->set_mode = 0;
    }
}

void rfl_motor_speed_control(rfl_motor_s *motor)
{
    if (motor->controller_type == RFL_MOTOR_CONTROLLER_PID)
    {
        motor->max_speed_ = 0.0f;
        rflAngleUpdate(&motor->track_angle, RFL_ANGLE_FORMAT_DEGREE, 0.0f);

        PID_clear(&((rfl_motor_pid_controller_s *)(motor->controller))->angle_pid);
        rflAngleUpdate(&motor->track_angle, RFL_ANGLE_FORMAT_DEGREE, 0.0f);
        rflAngleUpdate(&motor->set_angle_, RFL_ANGLE_FORMAT_DEGREE, 0.0f);

        /* 速度环PID计算 */
        motor->control_output_ =
            PID_calc(&((rfl_motor_pid_controller_s *)(motor->controller))->speed_pid, motor->speed_, motor->set_speed_);
    }
}

void rfl_motor_angle_control(rfl_motor_s *motor)
{
    if (motor->controller_type == RFL_MOTOR_CONTROLLER_PID)
    {
        motor->max_speed_ = 0.0f;
        rflAngleUpdate(&motor->track_angle, RFL_ANGLE_FORMAT_DEGREE, 0.0f);

        /* 角度限幅 */
        if (motor->set_angle_.deg > motor->max_angle_.deg)
            rflAngleUpdate(&motor->set_angle_, RFL_ANGLE_FORMAT_DEGREE, motor->max_angle_.deg);
        else if (motor->set_angle_.deg < motor->min_angle_.deg)
            rflAngleUpdate(&motor->set_angle_, RFL_ANGLE_FORMAT_DEGREE, motor->min_angle_.deg);

        /* 角度环PID计算 */
        motor->set_speed_ = PID_calc(&((rfl_motor_pid_controller_s *)(motor->controller))->angle_pid, motor->angle_.deg,
                                     motor->set_angle_.deg);

        /* 速度环PID计算 */
        motor->control_output_ =
            PID_calc(&((rfl_motor_pid_controller_s *)(motor->controller))->speed_pid, motor->speed_, motor->set_speed_);
    }
    else if (motor->controller_type == RFL_MOTOR_CONTROLLER_UNITREE)
    {
        motor->max_speed_ = 0.0f;

        /* 角度限幅 */
        if (motor->set_angle_.deg > motor->max_angle_.deg)
            rflAngleUpdate(&motor->set_angle_, RFL_ANGLE_FORMAT_DEGREE, motor->max_angle_.deg);
        else if (motor->set_angle_.deg < motor->min_angle_.deg)
            rflAngleUpdate(&motor->set_angle_, RFL_ANGLE_FORMAT_DEGREE, motor->min_angle_.deg);

        ((unitree_motor_s *)(motor->driver))->set_shaft_angle = motor->set_angle_.rad;

        ((unitree_motor_s *)(motor->driver))->k_a = ((rfl_motor_unitree_controller_s *)(motor->controller))->k_angle;
        ((unitree_motor_s *)(motor->driver))->k_s = ((rfl_motor_unitree_controller_s *)(motor->controller))->k_speed;

        ((unitree_motor_s *)(motor->driver))->set_mode = 1;
    }
}

void rfl_motor_direction_control(rfl_motor_s *motor)
{
    if (motor->controller_type == RFL_MOTOR_CONTROLLER_PID)
    {
        motor->max_speed_ = 0.0f;
        rflAngleUpdate(&motor->track_angle, RFL_ANGLE_FORMAT_DEGREE, 0.0f);

        /* 角度限幅 */
        rflAngleUpdate(&motor->set_angle_, RFL_ANGLE_FORMAT_DEGREE,
                       rflFloatLoopConstrain(motor->set_angle_.deg, -DEG_PI, DEG_PI));

        /* 角度环PID计算 */
        motor->set_speed_ = PID_calc(&((rfl_motor_pid_controller_s *)(motor->controller))->angle_pid, 0.0f,
                                     rflFloatLoopConstrain(motor->set_angle_.deg - motor->angle_.deg, -DEG_PI, DEG_PI));

        /* 速度环PID计算 */
        motor->control_output_ =
            PID_calc(&((rfl_motor_pid_controller_s *)(motor->controller))->speed_pid, motor->speed_, motor->set_speed_);
    }
}

void rfl_motor_speed_angle_control(rfl_motor_s *motor)
{
    if (motor->controller_type == RFL_MOTOR_CONTROLLER_PID)
    {
        /* 角度限幅 */
        if (motor->set_angle_.deg > motor->max_angle_.deg)
            rflAngleUpdate(&motor->set_angle_, RFL_ANGLE_FORMAT_DEGREE, motor->max_angle_.deg);
        else if (motor->set_angle_.deg < motor->min_angle_.deg)
            rflAngleUpdate(&motor->set_angle_, RFL_ANGLE_FORMAT_DEGREE, motor->min_angle_.deg);

        float delta_angle = motor->set_angle_.deg - motor->angle_.deg;
        float max_step_angle_value = motor->max_speed_ * motor->control_period_factor;
        if (delta_angle > max_step_angle_value)
            rflAngleUpdate(&motor->track_angle, RFL_ANGLE_FORMAT_DEGREE, motor->angle_.deg + max_step_angle_value);
        else if (delta_angle < -max_step_angle_value)
            rflAngleUpdate(&motor->track_angle, RFL_ANGLE_FORMAT_DEGREE, motor->angle_.deg - max_step_angle_value);
        else
            rflAngleUpdate(&motor->track_angle, RFL_ANGLE_FORMAT_DEGREE, motor->set_angle_.deg);

        /* 角度限幅 */
        if (motor->track_angle.deg > motor->max_angle_.deg)
            rflAngleUpdate(&motor->track_angle, RFL_ANGLE_FORMAT_DEGREE, motor->max_angle_.deg);
        else if (motor->track_angle.deg < motor->min_angle_.deg)
            rflAngleUpdate(&motor->track_angle, RFL_ANGLE_FORMAT_DEGREE, motor->min_angle_.deg);

        /* 角度环PID计算 */
        motor->set_speed_ = PID_calc(&((rfl_motor_pid_controller_s *)(motor->controller))->angle_pid, motor->angle_.deg,
                                     motor->track_angle.deg);

        /* 速度环PID计算 */
        motor->control_output_ =
            PID_calc(&((rfl_motor_pid_controller_s *)(motor->controller))->speed_pid, motor->speed_, motor->set_speed_);
    }
    else if (motor->controller_type == RFL_MOTOR_CONTROLLER_UNITREE)
    {
        /* 角度限幅 */
        if (motor->set_angle_.deg > motor->max_angle_.deg)
            rflAngleUpdate(&motor->set_angle_, RFL_ANGLE_FORMAT_DEGREE, motor->max_angle_.deg);
        else if (motor->set_angle_.deg < motor->min_angle_.deg)
            rflAngleUpdate(&motor->set_angle_, RFL_ANGLE_FORMAT_DEGREE, motor->min_angle_.deg);

        float delta_angle = motor->set_angle_.deg - motor->angle_.deg;
        float max_step_angle_value = motor->max_speed_ * motor->control_period_factor;
        if (delta_angle > max_step_angle_value)
            rflAngleUpdate(&motor->track_angle, RFL_ANGLE_FORMAT_DEGREE, motor->angle_.deg + max_step_angle_value);
        else if (delta_angle < -max_step_angle_value)
            rflAngleUpdate(&motor->track_angle, RFL_ANGLE_FORMAT_DEGREE, motor->angle_.deg - max_step_angle_value);
        else
            rflAngleUpdate(&motor->track_angle, RFL_ANGLE_FORMAT_DEGREE, motor->set_angle_.deg);

        /* 角度限幅 */
        if (motor->track_angle.deg > motor->max_angle_.deg)
            rflAngleUpdate(&motor->track_angle, RFL_ANGLE_FORMAT_DEGREE, motor->max_angle_.deg);
        else if (motor->track_angle.deg < motor->min_angle_.deg)
            rflAngleUpdate(&motor->track_angle, RFL_ANGLE_FORMAT_DEGREE, motor->min_angle_.deg);

        ((unitree_motor_s *)(motor->driver))->set_shaft_angle = motor->track_angle.rad;

        ((unitree_motor_s *)(motor->driver))->k_a = ((rfl_motor_unitree_controller_s *)(motor->controller))->k_angle;
        ((unitree_motor_s *)(motor->driver))->k_s = ((rfl_motor_unitree_controller_s *)(motor->controller))->k_speed;

        ((unitree_motor_s *)(motor->driver))->set_mode = 1;
    }
}
