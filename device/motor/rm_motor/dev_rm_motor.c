#include "stdlib.h"

#include "dev_rm_motor.h"
#include "dev_rm_motor_controller.h"

#include "algo_data_limiting.h"

static void RflRmMotorUpdateState(void *rm_motor);
static void RflRmMotorUpdateControl(void *rm_motor);
static RflMotorError RflRmMotorResetPosition(void *rm_motor);
static RflMotorError RflRmMotorSetMode(void *rm_motor, RflMotorControlMode mode);
static float RflRmMotorGetControlOutput(RflRmMotor *self);

RflMotorError RflRmMotorGetDefaultConfig(RflRmMotorConfig *config, RflMotorType type,
                                         RflMotorControllerType controller_type)
{
    if (config == NULL)
        return RFL_MOTOR_NULL_POINTER;

    memset(config, 0, sizeof(RflRmMotorConfig));

    if (!(type >= RFL_MOTOR_RM_M2006 && type <= RFL_MOTOR_RM_GM6020))
    {
        return RFL_MOTOR_TYPE_MISMATCH;
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
        return RFL_MOTOR_CONTROLLER_TYPE_MISMATCH;
    }
    config->base.controller_type = controller_type;
    if (controller_type == RFL_RM_MOTOR_CONTROLLER_NORMAL_PID)
    {
        config->controller_param =
            (RflRmMotorControllerNormalPidParam *)malloc(sizeof(RflRmMotorControllerNormalPidParam));
        if (config->controller_param == NULL)
            return RFL_MOTOR_OUT_OF_MEMORY;
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

    return RFL_MOTOR_SUCCESS;
}

RflMotorError RflRmMotorInit(RflRmMotor *self, RflRmMotorConfig *config)
{
    memset(self, 0, sizeof(RflRmMotor));

    if (!(config->base.type >= RFL_MOTOR_RM_M2006 && config->base.type <= RFL_MOTOR_RM_GM6020))
    {
        config->base.type = RFL_MOTOR_UNDEFINED;
        return RFL_MOTOR_TYPE_MISMATCH;
    }
    RflBaseMotorInit(&self->base, &config->base);

    if (!(config->base.controller_type >= RFL_RM_MOTOR_CONTROLLER_NORMAL_PID &&
          config->base.controller_type <= RFL_RM_MOTOR_CONTROLLER_NORMAL_PID))
    {
        config->base.controller_type = RFL_MOTOR_CONTROLLER_UNDEFINED;
        return RFL_MOTOR_CONTROLLER_TYPE_MISMATCH;
    }
    self->base.controller_type_ = config->base.controller_type;

    if (self->base.controller_type_ == RFL_RM_MOTOR_CONTROLLER_NORMAL_PID)
    {
        self->base.controller_ = (RflRmMotorControllerNormalPid *)malloc(sizeof(RflRmMotorControllerNormalPid));
        if (self->base.controller_ == NULL)
            return RFL_MOTOR_OUT_OF_MEMORY;
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
    else
        self->torque_factor_ = 0.0f;

    self->max_rotor_turns_ = (int16_t)(RM_MOTOR_ROTOR_TURNS_RANGE_PARAM / self->ecd_to_effector_angle_factor_ - 2);
    self->min_rotor_turns_ = -(int16_t)(RM_MOTOR_ROTOR_TURNS_RANGE_PARAM / self->ecd_to_effector_angle_factor_ - 1);
    self->last_ecd_ = 4095;

    if (config->can_rx_data == NULL)
        return;
    self->can_rx_data_ = config->can_rx_data;

    self->GetControlOutput = RflRmMotorGetControlOutput;

    return RFL_MOTOR_SUCCESS;
}

static void RflRmMotorUpdateState(void *rm_motor)
{
    RflRmMotor *self = (RflRmMotor *)rm_motor;

    // // 更新电机反馈数据
    // decode_rm_motor_feedback(&rm_motor->feedback_, rm_motor->can_rx_data);

    // if (control_mode == CONTROL_MODE_SPEED)
    // {
    //     rm_motor->rotor_turns = 0;
    //     rm_motor->ecd_angle_offset = rm_motor->feedback_.ecd;
    // }
    // else if (control_mode != CONTROL_MODE_SPEED)
    // {
    //     // 跳变沿检测
    //     if (rm_motor->last_ecd - rm_motor->feedback_.ecd > RM_MOTOR_HALF_ECD_RANGE)
    //         rm_motor->rotor_turns++;
    //     else if (rm_motor->feedback_.ecd - rm_motor->last_ecd > RM_MOTOR_HALF_ECD_RANGE)
    //         rm_motor->rotor_turns--;

    //     // 单圈限幅
    //     if (angle_format == ANGLE_FORMAT_ABSOLUTE)
    //     {
    //         int16_t half_integer_transmission_ratio = (int16_t)(rm_motor->effector_transmission_ratio) / 2;
    //         if (rm_motor->rotor_turns > half_integer_transmission_ratio - 1 &&
    //             rm_motor->feedback_.ecd > rm_motor->ecd_angle_offset)
    //             rm_motor->rotor_turns = -half_integer_transmission_ratio;
    //         else if (rm_motor->rotor_turns < 1 - half_integer_transmission_ratio &&
    //                  rm_motor->feedback_.ecd < rm_motor->ecd_angle_offset)
    //             rm_motor->rotor_turns = half_integer_transmission_ratio;
    //     }
    // }

    // rm_motor->last_ecd = rm_motor->feedback_.ecd;

    // // 计算末端执行器转速
    // rm_motor->speed = (float)rm_motor->feedback_.speed_rpm * rm_motor->rpm_to_effector_speed_factor;

    // // 可测量最大圈数限制
    // if (rm_motor->rotor_turns > rm_motor->max_rotor_turns)
    //     rm_motor->rotor_turns = rm_motor->max_rotor_turns;
    // else if (rm_motor->rotor_turns < rm_motor->min_rotor_turns)
    //     rm_motor->rotor_turns = rm_motor->min_rotor_turns;

    // // 计算末端执行器角度 角度范围大约为 -10000°~10000°
    // rm_motor->ecd_angle =
    //     rm_motor->rotor_turns * RM_MOTOR_ECD_RANGE + rm_motor->feedback_.ecd - rm_motor->ecd_angle_offset;
    // rm_motor->deg_angle = (float)rm_motor->ecd_angle * rm_motor->ecd_to_effector_angle_factor;

    // // 计算电机转矩
    // rm_motor->torque = (float)rm_motor->feedback_.given_current * rm_motor->torque_factor;

    // // 计算电机温度
    // rm_motor->temperature = rm_motor->feedback_.temperate;

    // //////////////////////

    // motor->torque_ = ((rm_motor_s *)(motor->driver))->torque;

    // motor->internal_speed = ((rm_motor_s *)(motor->driver))->speed;

    // rflAngleUpdate(&motor->internal_angle, RFL_ANGLE_FORMAT_DEGREE, ((rm_motor_s *)(motor->driver))->deg_angle);

    // motor->temperature_ = (float)(((rm_motor_s *)(motor->driver))->temperature);

    // // 结算 考虑安装极性

    // motor->torque_ *= (motor->is_reversed ? -1.0f : 1.0f);

    // motor->internal_speed *= (motor->is_reversed ? -1.0f : 1.0f);
    // motor->speed_ = motor->external_speed == NULL ? motor->internal_speed : *motor->external_speed;

    // rflAngleUpdate(&motor->internal_angle, RFL_ANGLE_FORMAT_DEGREE,
    //                motor->internal_angle.deg * (motor->is_reversed ? -1.0f : 1.0f));
    // // 单圈角度处理
    // if (motor->angle_format == RFL_MOTOR_ANGLE_FORMAT_ABSOLUTE)
    //     rflAngleUpdate(&motor->internal_angle, RFL_ANGLE_FORMAT_DEGREE,
    //                    rflFloatLoopConstrain(motor->internal_angle.deg, -DEG_PI, DEG_PI));
    // rflAngleUpdate(&motor->angle_, RFL_ANGLE_FORMAT_DEGREE,
    //                motor->external_angle == NULL ? motor->internal_angle.deg : motor->external_angle->deg);
}

static void RflRmMotorUpdateControl(void *rm_motor)
{
}

static RflMotorError RflRmMotorResetPosition(void *rm_motor)
{
}

static RflMotorError RflRmMotorSetMode(void *rm_motor, RflMotorControlMode mode)
{
    RflRmMotor *self = (RflRmMotor *)rm_motor;

    if (mode == RFL_MOTOR_CONTROL_MODE_NO_FORCE)
    {
        self->base.mode_ = mode;
        RflRmMotorControllerNormalPid *controller = (RflRmMotorControllerNormalPid *)self->base.controller_;
        PID_clear(&controller->position_pid);
        PID_clear(&controller->speed_pid);
        self->control_output_ = self->base.set_speed_ = 0.0f;
        self->base.track_position_ = self->base.set_position_ = self->base.position_;
    }
    else if (mode == RFL_MOTOR_CONTROL_MODE_SPEED && self->base.mode_ == RFL_MOTOR_CONTROL_MODE_NO_FORCE)
    {
        self->base.mode_ = mode;
        self->base.track_position_ = self->base.set_position_ = 0.0f;
    }
    else if (mode == RFL_MOTOR_CONTROL_MODE_ANGLE && self->base.mode_ == RFL_MOTOR_CONTROL_MODE_NO_FORCE)
    {
        self->base.mode_ = mode;
    }
    else
    {
        return RFL_MOTOR_SET_MODE_INVALID;
    }

    return RFL_MOTOR_SUCCESS;
}

static float RflRmMotorGetControlOutput(RflRmMotor *self)
{
    return self->control_output_;
}
