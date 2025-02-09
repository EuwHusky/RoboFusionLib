#include "dev_rm_motor_controller.h"

#include "dev_rm_motor.h"

#include "algo_angle.h"

void RflRmMotorControlValueReset(RflRmMotor *self)
{
    self->base.track_position_ = self->base.set_position_ = self->base.position_;
    self->control_output_ = self->base.set_speed_ = 0.0f;
}

void RflRmMotorControllerReset(RflRmMotor *self)
{
    if (self->base.controller_type_ == RFL_RM_MOTOR_CONTROLLER_NORMAL_PID)
    {
        RflRmMotorNormalPidReset(self);
    }
}

void RflRmMotorNormalPidReset(RflRmMotor *self)
{
    PID_clear(&((RflRmMotorControllerNormalPid *)(self->base.controller_))->speed_pid);
    PID_clear(&((RflRmMotorControllerNormalPid *)(self->base.controller_))->position_pid);
}

void RflRmMotorNormalPidNoForceControl(RflRmMotor *self)
{
    RflRmMotorControlValueReset(self);
    RflRmMotorNormalPidReset(self);
}

void RflRmMotorNormalPidSpeedControl(RflRmMotor *self)
{
    self->base.track_position_ = 0.0f;

    // 速度限幅
    if (self->base.set_speed_ > self->base.max_speed_)
        self->base.set_speed_ = self->base.max_speed_;
    else if (self->base.set_speed_ < -self->base.max_speed_)
        self->base.set_speed_ = -self->base.max_speed_;

    // 速度环PID计算
    self->control_output_ = PID_calc(&((RflRmMotorControllerNormalPid *)(self->base.controller_))->speed_pid,
                                     self->base.speed_, self->base.set_speed_);

    // 重置位置环
    PID_clear(&((RflRmMotorControllerNormalPid *)(self->base.controller_))->position_pid);
    self->base.track_position_ = self->base.set_position_ = 0.0f;
}

void RflRmMotorNormalPidCircledPositionControl(RflRmMotor *self)
{
    // 位置限幅
    if (self->base.set_position_ > self->base.max_position_)
        self->base.set_position_ = self->base.max_position_;
    else if (self->base.set_position_ < self->base.min_position_)
        self->base.set_position_ = self->base.min_position_;

    // 跟随位置设定
    float max_position_step = self->base.max_speed_ * self->base.control_period_factor_;
    float delta_position = self->base.set_position_ - self->base.position_;
    if (delta_position > max_position_step)
        self->base.track_position_ = self->base.position_ + max_position_step;
    else if (delta_position < -max_position_step)
        self->base.track_position_ = self->base.position_ - max_position_step;
    else
        self->base.track_position_ = self->base.set_position_;

    // 位置环PID计算
    self->base.set_speed_ = PID_calc(&((RflRmMotorControllerNormalPid *)(self->base.controller_))->position_pid,
                                     self->base.position_, self->base.track_position_);

    // 速度环PID计算
    self->control_output_ = PID_calc(&((RflRmMotorControllerNormalPid *)(self->base.controller_))->speed_pid,
                                     self->base.speed_, self->base.set_speed_);
}

void RflRmMotorNormalPidAbsolutePositionControl(RflRmMotor *self)
{
    // 位置限幅
    self->base.set_position_ = rflFloatLoopConstrain(self->base.set_position_, -RAD_PI, RAD_PI);

    // 跟随位置设定
    float max_position_step = self->base.max_speed_ * self->base.control_period_factor_;
    if (max_position_step < RAD_PI)
    {
        float delta_position = rflFloatLoopConstrain(self->base.set_position_ - self->base.position_, -RAD_PI, RAD_PI);
        if (delta_position > max_position_step)
            self->base.track_position_ =
                rflFloatLoopConstrain(self->base.position_ + max_position_step, -RAD_PI, RAD_PI);
        else if (delta_position < -max_position_step)
            rflFloatLoopConstrain(self->base.position_ - max_position_step, -RAD_PI, RAD_PI);
    }
    else if (max_position_step >= RAD_PI)
        self->base.track_position_ = self->base.set_position_;

    /* 角度环PID计算 */
    self->base.set_speed_ =
        PID_calc(&((RflRmMotorControllerNormalPid *)(self->base.controller_))->position_pid, 0.0f,
                 rflFloatLoopConstrain(self->base.track_position_ - self->base.position_, -DEG_PI, DEG_PI));

    /* 速度环PID计算 */
    self->control_output_ = PID_calc(&((RflRmMotorControllerNormalPid *)(self->base.controller_))->speed_pid,
                                     self->base.speed_, self->base.set_speed_);
}
