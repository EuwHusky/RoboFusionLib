#include "math.h"
#include "string.h"

#include "dev_motor.h"

#include "drv_can.h"

#include "algo_data_limiting.h"

/**
 * @brief 获取电机默认配置
 */
void rflMotorGetDefaultConfig(rfl_motor_config_s *motor_config, rfl_motor_type_e type)
{
    motor_config->type = type;
    motor_config->mode = RFL_MOTOR_CONTROL_MODE_NO_FORCE;
    motor_config->angle_format = RFL_MOTOR_ANGLE_FORMAT_CIRCLED;

    switch (motor_config->type)
    {
#if (RFL_DEV_MOTOR_RM_MOTOR == 1)
    case RFL_MOTOR_RM_M2006:
        motor_config->effector_transmission_ratio = RM_M2006_REDUCTION_RATIO;
        break;
    case RFL_MOTOR_RM_M3508:
        motor_config->effector_transmission_ratio = RM_M3508_REDUCTION_RATIO;
        break;
    case RFL_MOTOR_RM_GM6020:
        motor_config->effector_transmission_ratio = RM_GM6020_REDUCTION_RATIO;
        break;
#endif /* RFL_DEV_MOTOR_RM_MOTOR == 1 */

    default:
        break;
    }

    motor_config->max_accle = 1.0f;

    rflAngleUpdate(&motor_config->max_angle, RFL_ANGLE_FORMAT_DEGREE, RFL_MOTOR_DEFAULT_ANGLE_RANGE);
    rflAngleUpdate(&motor_config->min_angle, RFL_ANGLE_FORMAT_DEGREE, -RFL_MOTOR_DEFAULT_ANGLE_RANGE);

    switch (motor_config->type)
    {
#if (RFL_DEV_MOTOR_RM_MOTOR == 1)
    case RFL_MOTOR_RM_M2006:
        motor_config->speed_pid_kp = RFL_MOTOR_RM_M2006_DEFAULT_SPEED_PID_KP;
        motor_config->speed_pid_ki = RFL_MOTOR_RM_M2006_DEFAULT_SPEED_PID_KI;
        motor_config->speed_pid_kd = RFL_MOTOR_RM_M2006_DEFAULT_SPEED_PID_KD;
        motor_config->speed_pid_max_iout = RFL_MOTOR_RM_M2006_DEFAULT_SPEED_PID_MAX_IOUT;
        motor_config->speed_pid_max_out = RFL_MOTOR_RM_M2006_DEFAULT_SPEED_PID_MAX_OUT;
        motor_config->angle_pid_kp = RFL_MOTOR_RM_M2006_DEFAULT_ANGLE_PID_KP;
        motor_config->angle_pid_ki = RFL_MOTOR_RM_M2006_DEFAULT_ANGLE_PID_KI;
        motor_config->angle_pid_kd = RFL_MOTOR_RM_M2006_DEFAULT_ANGLE_PID_KD;
        motor_config->angle_pid_max_iout = RFL_MOTOR_RM_M2006_DEFAULT_ANGLE_PID_MAX_IOUT;
        motor_config->angle_pid_max_out = RFL_MOTOR_RM_M2006_DEFAULT_ANGLE_PID_MAX_OUT;
        break;
    case RFL_MOTOR_RM_M3508:
        motor_config->speed_pid_kp = RFL_MOTOR_RM_M3508_DEFAULT_SPEED_PID_KP;
        motor_config->speed_pid_ki = RFL_MOTOR_RM_M3508_DEFAULT_SPEED_PID_KI;
        motor_config->speed_pid_kd = RFL_MOTOR_RM_M3508_DEFAULT_SPEED_PID_KD;
        motor_config->speed_pid_max_iout = RFL_MOTOR_RM_M3508_DEFAULT_SPEED_PID_MAX_IOUT;
        motor_config->speed_pid_max_out = RFL_MOTOR_RM_M3508_DEFAULT_SPEED_PID_MAX_OUT;
        motor_config->angle_pid_kp = RFL_MOTOR_RM_M3508_DEFAULT_ANGLE_PID_KP;
        motor_config->angle_pid_ki = RFL_MOTOR_RM_M3508_DEFAULT_ANGLE_PID_KI;
        motor_config->angle_pid_kd = RFL_MOTOR_RM_M3508_DEFAULT_ANGLE_PID_KD;
        motor_config->angle_pid_max_iout = RFL_MOTOR_RM_M3508_DEFAULT_ANGLE_PID_MAX_IOUT;
        motor_config->angle_pid_max_out = RFL_MOTOR_RM_M3508_DEFAULT_ANGLE_PID_MAX_OUT;
        break;
    case RFL_MOTOR_RM_GM6020:
        motor_config->speed_pid_kp = RFL_MOTOR_RM_GM6020_DEFAULT_SPEED_PID_KP;
        motor_config->speed_pid_ki = RFL_MOTOR_RM_GM6020_DEFAULT_SPEED_PID_KI;
        motor_config->speed_pid_kd = RFL_MOTOR_RM_GM6020_DEFAULT_SPEED_PID_KD;
        motor_config->speed_pid_max_iout = RFL_MOTOR_RM_GM6020_DEFAULT_SPEED_PID_MAX_IOUT;
        motor_config->speed_pid_max_out = RFL_MOTOR_RM_GM6020_DEFAULT_SPEED_PID_MAX_OUT;
        motor_config->angle_pid_kp = RFL_MOTOR_RM_GM6020_DEFAULT_ANGLE_PID_KP;
        motor_config->angle_pid_ki = RFL_MOTOR_RM_GM6020_DEFAULT_ANGLE_PID_KI;
        motor_config->angle_pid_kd = RFL_MOTOR_RM_GM6020_DEFAULT_ANGLE_PID_KD;
        motor_config->angle_pid_max_iout = RFL_MOTOR_RM_GM6020_DEFAULT_ANGLE_PID_MAX_IOUT;
        motor_config->angle_pid_max_out = RFL_MOTOR_RM_GM6020_DEFAULT_ANGLE_PID_MAX_OUT;
        break;
#endif /* RFL_DEV_MOTOR_RM_MOTOR == 1 */

    default:
        motor_config->speed_pid_kp = RFL_MOTOR_DEFAULT_SPEED_PID_KP;
        motor_config->speed_pid_ki = RFL_MOTOR_DEFAULT_SPEED_PID_KI;
        motor_config->speed_pid_kd = RFL_MOTOR_DEFAULT_SPEED_PID_KD;
        motor_config->speed_pid_max_iout = RFL_MOTOR_DEFAULT_SPEED_PID_MAX_IOUT;
        motor_config->speed_pid_max_out = RFL_MOTOR_DEFAULT_SPEED_PID_MAX_OUT;
        motor_config->angle_pid_kp = RFL_MOTOR_DEFAULT_ANGLE_PID_KP;
        motor_config->angle_pid_ki = RFL_MOTOR_DEFAULT_ANGLE_PID_KI;
        motor_config->angle_pid_kd = RFL_MOTOR_DEFAULT_ANGLE_PID_KD;
        motor_config->angle_pid_max_iout = RFL_MOTOR_DEFAULT_ANGLE_PID_MAX_IOUT;
        motor_config->angle_pid_max_out = RFL_MOTOR_DEFAULT_ANGLE_PID_MAX_OUT;
        break;
    }

    motor_config->external_speed = NULL;
    motor_config->external_angle = NULL;

#if (RFL_DEV_MOTOR_RM_MOTOR == 1)
    motor_config->can_handle_id = 1;
    motor_config->can_id = 0x201;
#endif /* RFL_DEV_MOTOR_RM_MOTOR == 1 */
}

/**
 * @brief 初始化电机
 */
void rflMotorInit(rfl_motor_s *motor, rfl_motor_config_s *motor_config)
{
#if (RFL_DEV_MOTOR_RM_MOTOR == 1)
    int16_t rm_motor_rotor_turns_range = 1;
#endif /* RFL_DEV_MOTOR_RM_MOTOR == 1 */

    memset(motor, 0, sizeof(rfl_motor_s));

    /* 基础参数 */

    motor->type = motor_config->type;

    motor->mode_ = motor_config->mode;

    motor->angle_format = motor_config->angle_format;
    if (motor->mode_ == RFL_MOTOR_CONTROL_MODE_ANGLE)
        motor->angle_format = RFL_MOTOR_ANGLE_FORMAT_CIRCLED;
    else if (motor->mode_ == RFL_MOTOR_CONTROL_MODE_DIRECTION)
        motor->angle_format = RFL_MOTOR_ANGLE_FORMAT_ABSOLUTE;

    motor->effector_transmission_ratio_ = motor_config->effector_transmission_ratio;

    /* 控制量 */

    motor->set_speed_ = 0.0f;
    motor->max_accle_ = motor_config->max_accle;

    rflAngleUpdate(&motor->set_angle_, RFL_ANGLE_FORMAT_DEGREE, 0.0f);
    rflAngleUpdate(&motor->max_angle_, RFL_ANGLE_FORMAT_DEGREE, motor_config->max_angle.deg);
    rflAngleUpdate(&motor->min_angle_, RFL_ANGLE_FORMAT_DEGREE, motor_config->min_angle.deg);

    const float speed_pid_param[3] = {motor_config->speed_pid_kp, motor_config->speed_pid_ki,
                                      motor_config->speed_pid_kd};
    PID_init(&motor->speed_pid, PID_POSITION, speed_pid_param, motor_config->speed_pid_max_out,
             motor_config->speed_pid_max_iout);
    const float angle_pid_param[3] = {motor_config->angle_pid_kp, motor_config->angle_pid_ki,
                                      motor_config->angle_pid_kd};
    PID_init(&motor->angle_pid, PID_POSITION, angle_pid_param, motor_config->angle_pid_max_out,
             motor_config->angle_pid_max_iout);

    switch (motor->type)
    {
#if (RFL_DEV_MOTOR_RM_MOTOR == 1)
    case RFL_MOTOR_RM_M2006:
    case RFL_MOTOR_RM_M3508:
    case RFL_MOTOR_RM_GM6020:
        motor->motor_driver = (rm_motor_s *)malloc(sizeof(rm_motor_s));

        ((rm_motor_s *)(motor->motor_driver))->effector_transmission_ratio =
            motor->effector_transmission_ratio_;
        ((rm_motor_s *)(motor->motor_driver))->ecd_to_effector_angle_factor =
            RM_MOTOR_ECD_TO_EFFECTOR_ANGLE_FACTOR / motor->effector_transmission_ratio_;
        ((rm_motor_s *)(motor->motor_driver))->rpm_to_effector_speed_factor =
            RM_MOTOR_RPM_TO_EFFECTOR_SPEED_FACTOR / motor->effector_transmission_ratio_;

        rm_motor_rotor_turns_range = RM_MOTOR_ROTOR_TURNS_RANGE_PARAM /
                                     ((rm_motor_s *)(motor->motor_driver))->ecd_to_effector_angle_factor;
        ((rm_motor_s *)(motor->motor_driver))->max_rotor_turns = rm_motor_rotor_turns_range - 2;
        ((rm_motor_s *)(motor->motor_driver))->min_rotor_turns = -(rm_motor_rotor_turns_range - 1);

        if (motor_config->can_handle_id == 1)
        {
            ((rm_motor_s *)(motor->motor_driver))->can_rx_data = rflCan1AddRxMessageBox(motor_config->can_id);
        }
        else if (motor_config->can_handle_id == 2)
        {
            ((rm_motor_s *)(motor->motor_driver))->can_rx_data = rflCan2AddRxMessageBox(motor_config->can_id);
        }

        rm_motor_init((rm_motor_s *)(motor->motor_driver));

        break;
#endif /* RFL_DEV_MOTOR_RM_MOTOR == 1 */

    default:
        break;
    }

    /* 状态量 */

    motor->external_speed = motor_config->external_speed;
    motor->external_angle = motor_config->external_angle;
}

/**
 * @brief 更新电机状态量
 */
void rflMotorUpdateStatus(rfl_motor_s *motor)
{
    switch (motor->type)
    {
#if (RFL_DEV_MOTOR_RM_MOTOR == 1)
    case RFL_MOTOR_RM_M2006:
    case RFL_MOTOR_RM_M3508:
    case RFL_MOTOR_RM_GM6020:

        rm_motor_update_status((rm_motor_s *)(motor->motor_driver), (uint8_t)motor->mode_,
                               (uint8_t)motor->angle_format);

        if (motor->external_speed == NULL)
            motor->speed = ((rm_motor_s *)(motor->motor_driver))->speed;
        else
            motor->speed = *motor->external_speed;

        if (motor->external_angle == NULL)
            rflAngleUpdate(&motor->angle, RFL_ANGLE_FORMAT_DEGREE,
                           ((rm_motor_s *)(motor->motor_driver))->deg_angle);
        else
            rflAngleUpdate(&motor->angle, RFL_ANGLE_FORMAT_DEGREE, motor->external_angle->deg);

        break;
#endif /* RFL_DEV_MOTOR_RM_MOTOR == 1 */

    default:
        break;
    }
}

/**
 * @brief 更新电机控制量
 */
float rflMotorUpdataControl(rfl_motor_s *motor)
{
    if (motor->mode_ == RFL_MOTOR_CONTROL_MODE_NO_FORCE)
    {
        PID_clear(&motor->angle_pid);
        PID_clear(&motor->speed_pid);
        motor->control_output = 0.0f;

        return 0.0f;
    }
    else if (motor->mode_ == RFL_MOTOR_CONTROL_MODE_ANGLE)
    {
        /* 角度限幅 */
        if (motor->set_angle_.deg > motor->max_angle_.deg)
            rflAngleUpdate(&motor->set_angle_, RFL_ANGLE_FORMAT_DEGREE, motor->max_angle_.deg);
        else if (motor->set_angle_.deg < motor->min_angle_.deg)
            rflAngleUpdate(&motor->set_angle_, RFL_ANGLE_FORMAT_DEGREE, motor->min_angle_.deg);

        /* 角度环PID计算 */
        motor->set_speed_ = PID_calc(&motor->angle_pid, motor->angle.deg, motor->set_angle_.deg);
    }
    else if (motor->mode_ == RFL_MOTOR_CONTROL_MODE_DIRECTION)
    {
        /* 角度限幅 */
        rflAngleUpdate(&motor->set_angle_, RFL_ANGLE_FORMAT_DEGREE,
                       rflFloatLoopConstrain(motor->set_angle_.deg, -DEG_PI, DEG_PI));

        /* 角度环PID计算 */
        motor->set_speed_ = PID_calc(&motor->angle_pid, 0.0f,
                                     rflFloatLoopConstrain(motor->set_angle_.deg - motor->angle.deg, -DEG_PI, DEG_PI));
    }

    if (motor->mode_ == RFL_MOTOR_CONTROL_MODE_SPEED || motor->mode_ == RFL_MOTOR_CONTROL_MODE_ANGLE ||
        motor->mode_ == RFL_MOTOR_CONTROL_MODE_DIRECTION)
    {
        /* 转动加速度限幅 */
        if (motor->set_speed_ - motor->speed > motor->max_accle_)
            motor->set_speed_ = motor->speed + motor->max_accle_;
        else if (motor->set_speed_ - motor->speed < -motor->max_accle_)
            motor->set_speed_ = motor->speed - motor->max_accle_;

        /* 速度环PID计算 */
        motor->control_output = PID_calc(&motor->speed_pid, motor->speed, motor->set_speed_);

        return motor->control_output;
    }

    return 0.0f;
}

/**
 * @brief 获取电机当前速度
 */
float rflMotorGetSpeed(rfl_motor_s *motor)
{
    return motor->speed;
}
/**
 * @brief 获取电机当前角度
 */
rfl_angle_s *rflMotorGetAngle(rfl_motor_s *motor)
{
    return &motor->angle;
}
/**
 * @brief 获取电机当前输出
 */
float rflMotorGetOutput(rfl_motor_s *motor)
{
    return motor->control_output;
}

/**
 * @brief 设置电机控制模式
 */
void rflMotorSetMode(rfl_motor_s *motor, rfl_motor_control_mode_e mode)
{
    motor->mode_ = mode;

    if (motor->mode_ == RFL_MOTOR_CONTROL_MODE_ANGLE)
        motor->angle_format = RFL_MOTOR_ANGLE_FORMAT_CIRCLED;
    else if (motor->mode_ == RFL_MOTOR_CONTROL_MODE_DIRECTION)
        motor->angle_format = RFL_MOTOR_ANGLE_FORMAT_ABSOLUTE;

    PID_clear(&motor->angle_pid);
    PID_clear(&motor->speed_pid);
    motor->control_output = motor->set_speed_ = 0.0f;
    rflAngleUpdate(&motor->set_angle_, RFL_ANGLE_FORMAT_DEGREE, 0.0f);
}
/**
 * @brief 设置电机预期角速度
 */
void rflMotorSetSpeed(rfl_motor_s *motor, float set_speed)
{
    motor->set_speed_ = set_speed;
}
/**
 * @brief 设置电机预期角加速度
 */
void rflMotorSetAccle(rfl_motor_s *motor, float set_accle)
{
    motor->max_accle_ = set_accle;
}
/**
 * @brief 设置电机预期角度
 */
void rflMotorSetAngle(rfl_motor_s *motor, rfl_angle_s set_angle)
{
    rflAngleUpdate(&motor->set_angle_, RFL_ANGLE_FORMAT_DEGREE, set_angle.deg);
}
/**
 * @brief 设置电机角度范围
 */
void rflMotorSetAngleLimit(rfl_motor_s *motor, rfl_angle_s max_angle, rfl_angle_s min_angle)
{
    rflAngleUpdate(&motor->max_angle_, RFL_ANGLE_FORMAT_DEGREE, max_angle.deg);
    rflAngleUpdate(&motor->min_angle_, RFL_ANGLE_FORMAT_DEGREE, min_angle.deg);
}

/**
 * @brief 重置电机零位，将当前位置设为零位
 */
void rflMotorResetAngle(rfl_motor_s *motor)
{
    switch (motor->type)
    {
#if (RFL_DEV_MOTOR_RM_MOTOR == 1)
    case RFL_MOTOR_RM_M2006:
    case RFL_MOTOR_RM_M3508:
    case RFL_MOTOR_RM_GM6020:
        rm_motor_reset_angle((rm_motor_s *)(motor->motor_driver));
        break;
#endif /* RFL_DEV_MOTOR_RM_MOTOR == 1 */

    default:
        break;
    }
}
