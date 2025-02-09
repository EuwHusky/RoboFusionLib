#include "stdlib.h"

#include "dev_rm_motor.h"
#include "dev_rm_motor_controller.h"

#include "algo_angle.h"
#include "algo_data_limiting.h"

static RflResult RflRmMotorUpdateState(void *rm_motor);
static RflResult RflRmMotorUpdateControl(void *rm_motor);
static RflResult RflRmMotorResetPosition(void *rm_motor, float position);
static RflResult RflRmMotorSetMode(void *rm_motor, RflMotorControlMode mode);
static RflResult RflRmMotorGetControlOutput(void *rm_motor);

RflResult RflRmMotorGetDefaultConfig(RflRmMotorConfig *config, RflMotorType type,
                                     RflMotorControllerType controller_type)
{
    RflResult ret = {0};

    if (config == NULL)
    {
        ret.error = RFL_ERROR_NULL_POINTER;
        return ret;
    }

    memset(config, 0, sizeof(RflRmMotorConfig));

    if (!(type >= RFL_MOTOR_RM_M2006 && type <= RFL_MOTOR_RM_GM6020))
    {
        ret.error = RFL_ERROR_INVALID_ARG;
        return ret;
    }
    RflBaseMotorGetDefaultConfig(&config->base, type);

    if (type == RFL_MOTOR_RM_M2006)
        config->base.position_conversion_factor = RM_M2006_REDUCTION_RATIO;
    else if (type == RFL_MOTOR_RM_M3508)
        config->base.position_conversion_factor = RM_M3508_REDUCTION_RATIO;
    else if (type == RFL_MOTOR_RM_GM6020)
        config->base.position_conversion_factor = RM_GM6020_REDUCTION_RATIO;

    if (!(controller_type >= RFL_RM_MOTOR_CONTROLLER_NORMAL_PID &&
          controller_type <= RFL_RM_MOTOR_CONTROLLER_NORMAL_PID))
    {
        ret.error = RFL_ERROR_INVALID_ARG;
        return ret;
    }
    config->base.controller_type = controller_type;
    if (controller_type == RFL_RM_MOTOR_CONTROLLER_NORMAL_PID)
    {
        config->controller_param =
            (RflRmMotorControllerNormalPidParam *)malloc(sizeof(RflRmMotorControllerNormalPidParam));
        if (config->controller_param == NULL)
        {
            ret.error = RFL_ERROR_OUT_OF_MEMORY;
            return ret;
        }
        RflRmMotorControllerNormalPidParam *controller_param =
            (RflRmMotorControllerNormalPidParam *)config->controller_param;
        if (type == RFL_MOTOR_RM_M2006)
        {
            controller_param->speed_pid_kp = RFL_MOTOR_RM_M2006_DEFAULT_SPEED_PID_KP;
            controller_param->speed_pid_ki = RFL_MOTOR_RM_M2006_DEFAULT_SPEED_PID_KI;
            controller_param->speed_pid_kd = RFL_MOTOR_RM_M2006_DEFAULT_SPEED_PID_KD;
            controller_param->speed_pid_max_iout = RFL_MOTOR_RM_M2006_DEFAULT_SPEED_PID_MAX_IOUT;
            controller_param->speed_pid_max_out = RFL_MOTOR_RM_M2006_DEFAULT_SPEED_PID_MAX_OUT;
            controller_param->position_pid_kp = RFL_MOTOR_RM_M2006_DEFAULT_POSITION_PID_KP;
            controller_param->position_pid_ki = RFL_MOTOR_RM_M2006_DEFAULT_POSITION_PID_KI;
            controller_param->position_pid_kd = RFL_MOTOR_RM_M2006_DEFAULT_POSITION_PID_KD;
            controller_param->position_pid_max_iout = RFL_MOTOR_RM_M2006_DEFAULT_POSITION_PID_MAX_IOUT;
            controller_param->position_pid_max_out = RFL_MOTOR_RM_M2006_DEFAULT_POSITION_PID_MAX_OUT;
        }
        else if (type == RFL_MOTOR_RM_M3508)
        {
            controller_param->speed_pid_kp = RFL_MOTOR_RM_M3508_DEFAULT_SPEED_PID_KP;
            controller_param->speed_pid_ki = RFL_MOTOR_RM_M3508_DEFAULT_SPEED_PID_KI;
            controller_param->speed_pid_kd = RFL_MOTOR_RM_M3508_DEFAULT_SPEED_PID_KD;
            controller_param->speed_pid_max_iout = RFL_MOTOR_RM_M3508_DEFAULT_SPEED_PID_MAX_IOUT;
            controller_param->speed_pid_max_out = RFL_MOTOR_RM_M3508_DEFAULT_SPEED_PID_MAX_OUT;
            controller_param->position_pid_kp = RFL_MOTOR_RM_M3508_DEFAULT_POSITION_PID_KP;
            controller_param->position_pid_ki = RFL_MOTOR_RM_M3508_DEFAULT_POSITION_PID_KI;
            controller_param->position_pid_kd = RFL_MOTOR_RM_M3508_DEFAULT_POSITION_PID_KD;
            controller_param->position_pid_max_iout = RFL_MOTOR_RM_M3508_DEFAULT_POSITION_PID_MAX_IOUT;
            controller_param->position_pid_max_out = RFL_MOTOR_RM_M3508_DEFAULT_POSITION_PID_MAX_OUT;
        }
        else if (type == RFL_MOTOR_RM_GM6020)
        {
            controller_param->speed_pid_kp = RFL_MOTOR_RM_GM6020_DEFAULT_SPEED_PID_KP;
            controller_param->speed_pid_ki = RFL_MOTOR_RM_GM6020_DEFAULT_SPEED_PID_KI;
            controller_param->speed_pid_kd = RFL_MOTOR_RM_GM6020_DEFAULT_SPEED_PID_KD;
            controller_param->speed_pid_max_iout = RFL_MOTOR_RM_GM6020_DEFAULT_SPEED_PID_MAX_IOUT;
            controller_param->speed_pid_max_out = RFL_MOTOR_RM_GM6020_DEFAULT_SPEED_PID_MAX_OUT;
            controller_param->position_pid_kp = RFL_MOTOR_RM_GM6020_DEFAULT_POSITION_PID_KP;
            controller_param->position_pid_ki = RFL_MOTOR_RM_GM6020_DEFAULT_POSITION_PID_KI;
            controller_param->position_pid_kd = RFL_MOTOR_RM_GM6020_DEFAULT_POSITION_PID_KD;
            controller_param->position_pid_max_iout = RFL_MOTOR_RM_GM6020_DEFAULT_POSITION_PID_MAX_IOUT;
            controller_param->position_pid_max_out = RFL_MOTOR_RM_GM6020_DEFAULT_POSITION_PID_MAX_OUT;
        }
    }

    config->can_rx_data = NULL;

    ret.error = RFL_SUCCESS;
    return ret;
}

RflResult RflRmMotorInit(RflRmMotor *self, RflRmMotorConfig *config)
{
    RflResult ret = {0};

    if (self == NULL || config == NULL)
    {
        ret.error = RFL_ERROR_NULL_POINTER;
        return ret;
    }

    memset(self, 0, sizeof(RflRmMotor));

    if (!(config->base.type >= RFL_MOTOR_RM_M2006 && config->base.type <= RFL_MOTOR_RM_GM6020))
    {
        config->base.type = RFL_MOTOR_UNDEFINED;
        ret.error = RFL_ERROR_INVALID_ARG;
        return ret;
    }
    RflBaseMotorInit(&self->base, &config->base);

    if (!(config->base.controller_type >= RFL_RM_MOTOR_CONTROLLER_NORMAL_PID &&
          config->base.controller_type <= RFL_RM_MOTOR_CONTROLLER_NORMAL_PID))
    {
        config->base.controller_type = RFL_MOTOR_CONTROLLER_UNDEFINED;
        ret.error = RFL_ERROR_INVALID_ARG;
        return ret;
    }
    self->base.controller_type_ = config->base.controller_type;

    if (self->base.controller_type_ == RFL_RM_MOTOR_CONTROLLER_NORMAL_PID)
    {
        self->base.controller_ = (RflRmMotorControllerNormalPid *)malloc(sizeof(RflRmMotorControllerNormalPid));
        if (self->base.controller_ == NULL)
        {
            ret.error = RFL_ERROR_OUT_OF_MEMORY;
            return ret;
        }
        RflRmMotorControllerNormalPid *controller = (RflRmMotorControllerNormalPid *)self->base.controller_;
        memset(controller, 0, sizeof(RflRmMotorControllerNormalPid));
        RflRmMotorControllerNormalPidParam *controller_param =
            (RflRmMotorControllerNormalPidParam *)config->controller_param;
        const float speed_pid_param[3] = {controller_param->speed_pid_kp, controller_param->speed_pid_ki,
                                          controller_param->speed_pid_kd};
        PID_init(&controller->speed_pid, PID_POSITION, speed_pid_param, controller_param->speed_pid_max_out,
                 controller_param->speed_pid_max_iout);
        const float position_pid_param[3] = {controller_param->position_pid_kp, controller_param->position_pid_ki,
                                             controller_param->position_pid_kd};
        PID_init(&controller->position_pid, PID_POSITION, position_pid_param, controller_param->position_pid_max_out,
                 controller_param->position_pid_max_iout);
    }

    self->base.external_speed_ = config->base.external_speed;
    self->base.external_position_ = config->base.external_position;

    self->base.UpdateState = RflRmMotorUpdateState;
    self->base.UpdateControl = RflRmMotorUpdateControl;
    self->base.ResetPosition = RflRmMotorResetPosition;
    self->base.SetMode = RflRmMotorSetMode;

    self->ecd_to_effector_angle_factor_ =
        RM_MOTOR_ECD_TO_EFFECTOR_ANGLE_FACTOR / self->base.position_conversion_factor_;
    self->rpm_to_effector_speed_factor_ =
        RM_MOTOR_RPM_TO_EFFECTOR_SPEED_FACTOR / self->base.position_conversion_factor_;
    if (config->base.type == RFL_MOTOR_RM_M2006)
        self->torque_factor_ = RM_M2006_TORQUE_FACTOR;
    else if (config->base.type == RFL_MOTOR_RM_M3508)
        self->torque_factor_ = RM_M3508_TORQUE_FACTOR;
    else if (config->base.type == RFL_MOTOR_RM_GM6020)
        self->torque_factor_ = RM_GM6020_TORQUE_FACTOR;
    else
        self->torque_factor_ = 0.0f;

    self->max_rotor_turns_ = (int16_t)(RM_MOTOR_ROTOR_TURNS_RANGE_PARAM / self->ecd_to_effector_angle_factor_ - 2);
    self->min_rotor_turns_ = -(int16_t)(RM_MOTOR_ROTOR_TURNS_RANGE_PARAM / self->ecd_to_effector_angle_factor_ - 1);
    self->last_ecd_ = 4095;

    if (config->can_rx_data == NULL)
    {
        ret.error = RFL_ERROR_NULL_POINTER;
        return ret;
    }
    self->can_rx_data_ = config->can_rx_data;

    self->GetControlOutput = RflRmMotorGetControlOutput;

    ret.error = RFL_SUCCESS;
    return ret;
}

static RflResult RflRmMotorUpdateState(void *rm_motor)
{
    RflResult ret = {0};
    if (rm_motor == NULL)
    {
        ret.error = RFL_ERROR_NULL_POINTER;
        return ret;
    }

    RflRmMotor *self = (RflRmMotor *)rm_motor;

    // 更新电机反馈数据
    DecodeRmMotorFeedback(&self->feedback_, self->can_rx_data_);

    if (self->base.mode_ == RFL_MOTOR_CONTROL_MODE_SPEED)
    {
        self->rotor_turns_ = 0;
        self->ecd_angle_offset_ = self->feedback_.ecd;
    }
    else if (self->base.mode_ != RFL_MOTOR_CONTROL_MODE_SPEED)
    {
        // 跳变沿检测
        if (self->last_ecd_ - self->feedback_.ecd > RM_MOTOR_HALF_ECD_RANGE)
            self->rotor_turns_++;
        else if (self->feedback_.ecd - self->last_ecd_ > RM_MOTOR_HALF_ECD_RANGE)
            self->rotor_turns_--;

        // 单圈限幅
        if (self->base.angle_format_ == RFL_MOTOR_ANGLE_FORMAT_ABSOLUTE)
        {
            int16_t half_integer_transmission_ratio = (int16_t)(self->ecd_to_effector_angle_factor_) / 2;
            if (self->rotor_turns_ > half_integer_transmission_ratio - 1 &&
                self->feedback_.ecd > self->ecd_angle_offset_)
                self->rotor_turns_ = -half_integer_transmission_ratio;
            else if (self->rotor_turns_ < 1 - half_integer_transmission_ratio &&
                     self->feedback_.ecd < self->ecd_angle_offset_)
                self->rotor_turns_ = half_integer_transmission_ratio;
        }
    }

    self->last_ecd_ = self->feedback_.ecd;

    // 计算末端执行器转速
    self->base.internal_speed_ = (float)self->feedback_.speed_rpm * self->rpm_to_effector_speed_factor_;

    // 可测量最大圈数限制
    if (self->rotor_turns_ > self->max_rotor_turns_)
        self->rotor_turns_ = self->max_rotor_turns_;
    else if (self->rotor_turns_ < self->min_rotor_turns_)
        self->rotor_turns_ = self->min_rotor_turns_;

    // 计算末端执行器角度 角度范围大约为 -10000°~10000°
    self->ecd_angle_ = self->rotor_turns_ * RM_MOTOR_ECD_RANGE + self->feedback_.ecd - self->ecd_angle_offset_;
    self->base.internal_position_ =
        (float)self->ecd_angle_ * self->ecd_to_effector_angle_factor_ * DEGREE_TO_RADIAN_FACTOR;
    // 单圈角度处理
    if (self->base.angle_format_ == RFL_MOTOR_ANGLE_FORMAT_ABSOLUTE)
    {
        self->base.internal_position_ = rflFloatLoopConstrain(self->base.internal_position_, -RAD_PI, RAD_PI);
    }

    // 计算电机转矩
    self->base.torque_ = (float)self->feedback_.given_current * self->torque_factor_;

    // 计算电机温度
    self->base.temperature_ = self->feedback_.temperate;

    // 结算 考虑安装极性与外部数据源

    self->base.torque_ *= (self->base.is_reversed_ ? -1.0f : 1.0f);

    self->base.internal_speed_ *= (self->base.is_reversed_ ? -1.0f : 1.0f);
    self->base.speed_ = self->base.external_speed_ == NULL ? self->base.internal_speed_ : *self->base.external_speed_;

    self->base.internal_position_ *= (self->base.is_reversed_ ? -1.0f : 1.0f);
    self->base.position_ =
        self->base.external_position_ == NULL ? self->base.internal_position_ : *self->base.external_position_;

    ret.error = RFL_SUCCESS;
    return ret;
}

static RflResult RflRmMotorUpdateControl(void *rm_motor)
{
    RflResult ret = {0};
    if (rm_motor == NULL)
    {
        ret.error = RFL_ERROR_NULL_POINTER;
        return ret;
    }

    RflRmMotor *self = (RflRmMotor *)rm_motor;

    switch (self->base.mode_)
    {
    case RFL_MOTOR_CONTROL_MODE_NO_FORCE:
        RflRmMotorNormalPidNoForceControl(self);
        break;
    case RFL_MOTOR_CONTROL_MODE_SPEED:
        RflRmMotorNormalPidSpeedControl(self);
        break;
    case RFL_MOTOR_CONTROL_MODE_POSITION:
        if (self->base.angle_format_ == RFL_MOTOR_ANGLE_FORMAT_CIRCLED)
            RflRmMotorNormalPidCircledPositionControl(self);
        else if (self->base.angle_format_ == RFL_MOTOR_ANGLE_FORMAT_ABSOLUTE)
            RflRmMotorNormalPidAbsolutePositionControl(self);
        break;

    default:
        break;
    }

    self->control_output_ *= (self->base.is_reversed_ ? -1.0f : 1.0f);

    ret.error = RFL_SUCCESS;
    return ret;
}

static RflResult RflRmMotorResetPosition(void *rm_motor, float position)
{
    RflResult ret = {0};
    if (rm_motor == NULL)
    {
        ret.error = RFL_ERROR_NULL_POINTER;
        return ret;
    }

    RflRmMotor *self = (RflRmMotor *)rm_motor;

    position *= (self->base.is_reversed_ ? -1.0f : 1.0f);

    // 反向计算转子圈数和偏置 为了方便所以将偏置从无符号改为了有符号整形
    int32_t ecd_angle =
        (int32_t)((double)position * RADIAN_TO_DEGREE_FACTOR / (double)self->ecd_to_effector_angle_factor_);
    self->rotor_turns_ = ecd_angle / RM_MOTOR_ECD_RANGE;
    self->ecd_angle_offset_ = self->feedback_.ecd - (ecd_angle - self->rotor_turns_ * RM_MOTOR_ECD_RANGE);

    // 重新计算末端执行器角度
    self->ecd_angle_ = self->rotor_turns_ * RM_MOTOR_ECD_RANGE + self->feedback_.ecd - self->ecd_angle_offset_;
    self->base.internal_position_ =
        (float)self->ecd_angle_ * self->ecd_to_effector_angle_factor_ * DEGREE_TO_RADIAN_FACTOR;
    // 单圈角度处理
    if (self->base.angle_format_ == RFL_MOTOR_ANGLE_FORMAT_ABSOLUTE)
    {
        self->base.internal_position_ = rflFloatLoopConstrain(self->base.internal_position_, -RAD_PI, RAD_PI);
    }

    self->base.internal_position_ *= (self->base.is_reversed_ ? -1.0f : 1.0f);
    self->base.position_ =
        self->base.external_position_ == NULL ? self->base.internal_position_ : *self->base.external_position_;

    // 重置控制量
    RflRmMotorControlValueReset(self);
    RflRmMotorControllerReset(self);

    ret.error = RFL_SUCCESS;
    return ret;
}

static RflResult RflRmMotorSetMode(void *rm_motor, RflMotorControlMode mode)
{
    RflResult ret = {0};
    if (rm_motor == NULL)
    {
        ret.error = RFL_ERROR_NULL_POINTER;
        return ret;
    }

    RflRmMotor *self = (RflRmMotor *)rm_motor;

    if (mode == RFL_MOTOR_CONTROL_MODE_NO_FORCE)
    {
        self->base.mode_ = mode;
        RflRmMotorControlValueReset(self);
        RflRmMotorControllerReset(self);
    }
    else if (mode == RFL_MOTOR_CONTROL_MODE_SPEED && self->base.mode_ == RFL_MOTOR_CONTROL_MODE_NO_FORCE)
    {
        self->base.mode_ = mode;
        self->base.track_position_ = self->base.set_position_ = 0.0f;
    }
    else if (mode == RFL_MOTOR_CONTROL_MODE_POSITION && self->base.mode_ == RFL_MOTOR_CONTROL_MODE_NO_FORCE)
    {
        self->base.mode_ = mode;
    }
    else
    {
        ret.error = RFL_ERROR_INVALID_ARG;
    }

    ret.error = RFL_SUCCESS;
    return ret;
}

static RflResult RflRmMotorGetControlOutput(void *rm_motor)
{
    RflResult ret = {0};
    if (rm_motor == NULL)
    {
        ret.error = RFL_ERROR_NULL_POINTER;
        return ret;
    }

    ret.value.f = ((RflRmMotor *)rm_motor)->control_output_;
    ret.error = RFL_SUCCESS;

    return ret;
}
