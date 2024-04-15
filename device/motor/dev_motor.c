#include "math.h"
#include "stdlib.h"
#include "string.h"

#include "dev_motor.h"
#include "dev_motor_control.h"

#include "drv_can.h"

#include "algo_data_limiting.h"

/**
 * @brief 获取电机默认配置
 */
void rflMotorGetDefaultConfig(rfl_motor_config_s *motor_config, rfl_motor_type_e type,
                              rfl_motor_controller_type_e controller)
{
    memset(motor_config, 0, sizeof(rfl_motor_config_s));

    /* 基础参数 */
    motor_config->type = type;
    motor_config->controller_type = controller;
    motor_config->mode = RFL_MOTOR_CONTROL_MODE_NO_FORCE;

    motor_config->angle_format = RFL_MOTOR_ANGLE_FORMAT_CIRCLED;
    rflAngleUpdate(&motor_config->max_angle, RFL_ANGLE_FORMAT_DEGREE, RFL_MOTOR_DEFAULT_ANGLE_RANGE);
    rflAngleUpdate(&motor_config->min_angle, RFL_ANGLE_FORMAT_DEGREE, -RFL_MOTOR_DEFAULT_ANGLE_RANGE);

    motor_config->control_period_factor = RFL_MOTOR_DEFAULT_CONTROL_PERIOD_FACTOR;

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

#if (RFL_DEV_MOTOR_UNITREE_MOTOR == 1)
    case RFL_MOTOR_UNITREE_GO_M8010_6:
        motor_config->effector_transmission_ratio = UNITREE_GO_M8010_6_REDUCTION_RATIO;
        break;
#endif /* RFL_DEV_MOTOR_UNITREE_MOTOR == 1 */

    default:
        break;
    }

    motor_config->is_reversed = false;

    /* 控制量 */

    motor_config->max_speed = RFL_MOTOR_DEFAULT_MAX_SPEED;

    if (motor_config->controller_type == RFL_MOTOR_CONTROLLER_PID)
    {
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
    }
#if (RFL_DEV_MOTOR_UNITREE_MOTOR == 1)
    else if (motor_config->controller_type == RFL_MOTOR_CONTROLLER_UNITREE)
    {
        switch (motor_config->type)
        {
        case RFL_MOTOR_UNITREE_GO_M8010_6:
            motor_config->unitree_k_a = RFL_MOTOR_UNITREE_GO_M8010_6_K_ANGLE;
            motor_config->unitree_k_s = RFL_MOTOR_UNITREE_GO_M8010_6_K_SPEED;
            break;

        default:
            motor_config->unitree_k_a = RFL_MOTOR_UNITREE_DEFAULT_K_ANGLE;
            motor_config->unitree_k_s = RFL_MOTOR_UNITREE_DEFAULT_K_SPEED;
            break;
        }
    }
#endif /* RFL_DEV_MOTOR_UNITREE_MOTOR == 1 */

    /* 状态量 */

    motor_config->external_speed = NULL;
    motor_config->external_angle = NULL;

    /* 专有参数 */

#if (RFL_DEV_MOTOR_RM_MOTOR == 1)
    if (motor_config->type >= RFL_MOTOR_RM_M2006 && motor_config->type <= RFL_MOTOR_RM_GM6020)
    {
        motor_config->can_ordinal = 1;
        motor_config->master_can_id = 0x201;
    }
#endif /* RFL_DEV_MOTOR_RM_MOTOR == 1 */

#if (RFL_DEV_MOTOR_UNITREE_MOTOR == 1)
    motor_config->unitree_motor_id = 0;
#endif /* RFL_DEV_MOTOR_UNITREE_MOTOR == 1 */

#if (RFL_DEV_MOTOR_DAMIAO_MOTOR == 1)
    if (motor_config->type >= RFL_MOTOR_DM_J8009_2EC && motor_config->type <= RFL_MOTOR_DM_J8009_2EC)
    {
        motor_config->can_ordinal = 1;
        motor_config->master_can_id = 0x00;
        motor_config->slave_can_id = 0x01;

        motor_config->p_max = RFL_MOTOR_DAMIAO_DEFAULT_P_MAX;
        motor_config->v_max = RFL_MOTOR_DAMIAO_DEFAULT_V_MAX;
        motor_config->t_max = RFL_MOTOR_DAMIAO_DEFAULT_T_MAX;

        motor_config->damiao_motor_mode = DAMIAO_MOTOR_MODE_POS_SPEED;
    }
#endif /* RFL_DEV_MOTOR_RM_MOTOR == 1 */
}

/**
 * @brief 初始化电机
 */
void rflMotorInit(rfl_motor_s *motor, rfl_motor_config_s *motor_config)
{
    memset(motor, 0, sizeof(rfl_motor_s));

    /* 基础参数 */

    motor->type = motor_config->type;
    motor->controller_type = motor_config->controller_type;
    motor->mode_ = motor_config->mode;
    motor->last_mode = motor_config->mode;

    motor->angle_format = motor_config->angle_format;
    if (motor->mode_ == RFL_MOTOR_CONTROL_MODE_ANGLE)
        motor->angle_format = RFL_MOTOR_ANGLE_FORMAT_CIRCLED;
    else if (motor->mode_ == RFL_MOTOR_CONTROL_MODE_DIRECTION)
        motor->angle_format = RFL_MOTOR_ANGLE_FORMAT_ABSOLUTE;
    if (motor_config->max_angle.deg < motor_config->min_angle.deg)
        return;
#if (RFL_DEV_MOTOR_DAMIAO_MOTOR == 1)
    if (motor_config->type >= RFL_MOTOR_DM_J8009_2EC && motor_config->type <= RFL_MOTOR_DM_J8009_2EC)
    {
        rflAngleUpdate(&motor_config->max_angle, RFL_ANGLE_FORMAT_RADIAN,
                       rflFloatConstrain(motor_config->max_angle.rad, -motor_config->p_max, motor_config->p_max));
        rflAngleUpdate(&motor_config->min_angle, RFL_ANGLE_FORMAT_RADIAN,
                       rflFloatConstrain(motor_config->min_angle.rad, -motor_config->p_max, motor_config->p_max));
    }
#endif /* RFL_DEV_MOTOR_RM_MOTOR == 1 */
    rflAngleUpdate(&motor->max_angle_, RFL_ANGLE_FORMAT_DEGREE, motor_config->max_angle.deg);
    rflAngleUpdate(&motor->min_angle_, RFL_ANGLE_FORMAT_DEGREE, motor_config->min_angle.deg);

    motor->control_period_factor = motor_config->control_period_factor;

    motor->is_reversed = motor_config->is_reversed;

    /* 控制量 */

    motor->set_speed_ = 0.0f;
    motor->max_speed_ = motor_config->max_speed;

    rflAngleUpdate(&motor->set_angle_, RFL_ANGLE_FORMAT_DEGREE, 0.0f);
    rflAngleUpdate(&motor->track_angle, RFL_ANGLE_FORMAT_DEGREE, 0.0f);

    if (motor->controller_type == RFL_MOTOR_CONTROLLER_PID)
    {
        motor->controller = (rfl_motor_pid_controller_s *)malloc(sizeof(rfl_motor_pid_controller_s));
        memset(motor->controller, 0, sizeof(rfl_motor_pid_controller_s));

        const float speed_pid_param[3] = {motor_config->speed_pid_kp, motor_config->speed_pid_ki,
                                          motor_config->speed_pid_kd};
        PID_init(&((rfl_motor_pid_controller_s *)(motor->controller))->speed_pid, PID_POSITION, speed_pid_param,
                 motor_config->speed_pid_max_out, motor_config->speed_pid_max_iout);
        const float angle_pid_param[3] = {motor_config->angle_pid_kp, motor_config->angle_pid_ki,
                                          motor_config->angle_pid_kd};
        PID_init(&((rfl_motor_pid_controller_s *)(motor->controller))->angle_pid, PID_POSITION, angle_pid_param,
                 motor_config->angle_pid_max_out, motor_config->angle_pid_max_iout);
    }
    else if (motor->controller_type == RFL_MOTOR_CONTROLLER_UNITREE)
    {
        motor->controller = (rfl_motor_unitree_controller_s *)malloc(sizeof(rfl_motor_unitree_controller_s));
        memset(motor->controller, 0, sizeof(rfl_motor_unitree_controller_s));

        ((rfl_motor_unitree_controller_s *)(motor->controller))->k_angle = motor_config->unitree_k_a;
        ((rfl_motor_unitree_controller_s *)(motor->controller))->k_speed = motor_config->unitree_k_s;
    }

    switch (motor->type)
    {
#if (RFL_DEV_MOTOR_RM_MOTOR == 1)
    case RFL_MOTOR_RM_M2006:
    case RFL_MOTOR_RM_M3508:
    case RFL_MOTOR_RM_GM6020:

        motor->driver = (rm_motor_s *)malloc(sizeof(rm_motor_s));
        memset(motor->driver, 0, sizeof(rm_motor_s));

        ((rm_motor_s *)(motor->driver))->effector_transmission_ratio = motor_config->effector_transmission_ratio;
        if (motor_config->type == RFL_MOTOR_RM_M2006)
            ((rm_motor_s *)(motor->driver))->torque_factor = RM_M2006_TORQUE_FACTOR;
        else if (motor_config->type == RFL_MOTOR_RM_M3508)
            ((rm_motor_s *)(motor->driver))->torque_factor = RM_M3508_TORQUE_FACTOR;
        else
            ((rm_motor_s *)(motor->driver))->torque_factor = 0.0f;

        ((rm_motor_s *)(motor->driver))->can_ordinal = motor_config->can_ordinal;
        ((rm_motor_s *)(motor->driver))->master_can_id = motor_config->master_can_id;

        rm_motor_init((rm_motor_s *)(motor->driver));

        break;
#endif /* RFL_DEV_MOTOR_RM_MOTOR == 1 */

#if (RFL_DEV_MOTOR_UNITREE_MOTOR == 1)
    case RFL_MOTOR_UNITREE_GO_M8010_6:

        motor->driver = (unitree_motor_s *)malloc(sizeof(unitree_motor_s));
        memset(motor->driver, 0, sizeof(unitree_motor_s));

        ((unitree_motor_s *)(motor->driver))->effector_transmission_ratio = motor_config->effector_transmission_ratio;

        ((unitree_motor_s *)(motor->driver))->targer_id = motor_config->unitree_motor_id;

        ((unitree_motor_s *)(motor->driver))->feedback =
            unitree_motor_get_feedback_pointer(motor_config->unitree_motor_id);

        unitree_motor_init((unitree_motor_s *)(motor->driver));

        break;
#endif /* RFL_DEV_MOTOR_UNITREE_MOTOR == 1 */

#if (RFL_DEV_MOTOR_DAMIAO_MOTOR == 1)
    case RFL_MOTOR_DM_J8009_2EC:

        motor->driver = (damiao_motor_s *)malloc(sizeof(damiao_motor_s));
        memset(motor->driver, 0, sizeof(damiao_motor_s));

        damiao_motor_init((damiao_motor_s *)(motor->driver), motor_config->damiao_motor_mode, motor->is_reversed,
                          motor_config->can_ordinal, motor_config->master_can_id, motor_config->slave_can_id,
                          motor_config->p_max, motor_config->v_max, motor_config->t_max);

        break;
#endif /* RFL_DEV_MOTOR_DAMIAO_MOTOR == 1 */

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

        rm_motor_update_status((rm_motor_s *)(motor->driver), (uint8_t)motor->mode_, (uint8_t)motor->angle_format);

        motor->torque_ = ((rm_motor_s *)(motor->driver))->torque;

        if (motor->external_speed == NULL)
            motor->speed_ = ((rm_motor_s *)(motor->driver))->speed;
        else
            motor->speed_ = *motor->external_speed;

        if (motor->external_angle == NULL)
            rflAngleUpdate(&motor->angle_, RFL_ANGLE_FORMAT_DEGREE, ((rm_motor_s *)(motor->driver))->deg_angle);
        else
            rflAngleUpdate(&motor->angle_, RFL_ANGLE_FORMAT_DEGREE, motor->external_angle->deg);

        break;
#endif /* RFL_DEV_MOTOR_RM_MOTOR == 1 */

#if (RFL_DEV_MOTOR_UNITREE_MOTOR == 1)
    case RFL_MOTOR_UNITREE_GO_M8010_6:

        unitree_motor_update_status((unitree_motor_s *)(motor->driver));

        motor->torque_ = ((unitree_motor_s *)(motor->driver))->torque;

        if (motor->external_speed == NULL)
            motor->speed_ = ((unitree_motor_s *)(motor->driver))->shaft_speed;
        else
            motor->speed_ = *motor->external_speed;

        if (motor->external_angle == NULL)
            rflAngleUpdate(&motor->angle_, RFL_ANGLE_FORMAT_RADIAN, ((unitree_motor_s *)(motor->driver))->shaft_angle);
        else
            rflAngleUpdate(&motor->angle_, RFL_ANGLE_FORMAT_DEGREE, motor->external_angle->deg);

        break;
#endif /* RFL_DEV_MOTOR_UNITREE_MOTOR == 1 */

#if (RFL_DEV_MOTOR_DAMIAO_MOTOR == 1)
    case RFL_MOTOR_DM_J8009_2EC:

        damiao_motor_update_status((damiao_motor_s *)(motor->driver));

        motor->torque_ = ((damiao_motor_s *)(motor->driver))->torque;

        if (motor->external_speed == NULL)
            motor->speed_ = ((damiao_motor_s *)(motor->driver))->velocity;
        else
            motor->speed_ = *motor->external_speed;

        if (motor->external_angle == NULL)
            rflAngleUpdate(&motor->angle_, RFL_ANGLE_FORMAT_RADIAN, ((damiao_motor_s *)(motor->driver))->position);
        else
            rflAngleUpdate(&motor->angle_, RFL_ANGLE_FORMAT_DEGREE, motor->external_angle->deg);

        break;
#endif /* RFL_DEV_MOTOR_DAMIAO_MOTOR == 1 */

    default:
        break;
    }

    // 输入输出需考虑反转
    motor->torque_ *= (motor->is_reversed ? -1.0f : 1.0f);
    motor->speed_ *= (motor->is_reversed ? -1.0f : 1.0f);
    rflAngleUpdate(&motor->angle_, RFL_ANGLE_FORMAT_DEGREE, motor->angle_.deg * (motor->is_reversed ? -1.0f : 1.0f));
}

/**
 * @brief 更新电机控制量
 */
void rflMotorUpdateControl(rfl_motor_s *motor)
{
    switch (motor->controller_type)
    {
    case RFL_MOTOR_CONTROLLER_PID:
        switch (motor->mode_)
        {
        case RFL_MOTOR_CONTROL_MODE_NO_FORCE:
            rfl_motor_pid_no_force_control(motor);
            break;
        case RFL_MOTOR_CONTROL_MODE_SPEED:
            rfl_motor_pid_speed_control(motor);
            break;
        case RFL_MOTOR_CONTROL_MODE_ANGLE:
            rfl_motor_pid_angle_control(motor);
            break;
        case RFL_MOTOR_CONTROL_MODE_DIRECTION:
            rfl_motor_pid_direction_control(motor);
            break;
        case RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE:
            rfl_motor_pid_speed_angle_control(motor);
            break;

        default:
            break;
        }
        motor->control_output_ *= (motor->is_reversed ? -1.0f : 1.0f);
        break;

#if (RFL_DEV_MOTOR_UNITREE_MOTOR == 1)
    case RFL_MOTOR_CONTROLLER_UNITREE:
        switch (motor->mode_)
        {
        case RFL_MOTOR_CONTROL_MODE_NO_FORCE:
            rfl_motor_unitree_no_force_control(motor);
            break;
        case RFL_MOTOR_CONTROL_MODE_ANGLE:
            rfl_motor_unitree_angle_control(motor);
            break;
        case RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE:
            rfl_motor_unitree_speed_angle_control(motor);
            break;

        default:
            break;
        }
        ((unitree_motor_s *)(motor->driver))->set_shaft_angle *= (motor->is_reversed ? -1.0f : 1.0f);
        break;
#endif /* RFL_DEV_MOTOR_UNITREE_MOTOR == 1 */

#if (RFL_DEV_MOTOR_DAMIAO_MOTOR == 1)

    case RFL_MOTOR_CONTROLLER_DAMIAO:

        switch (motor->mode_)
        {
        case RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE:
            rfl_motor_damiao_speed_angle_control(motor);
            break;

        default:
            break;
        }
        // 已将电机安装极性属性内置到电机驱动 此处无需做反转处理
        break;
#endif /* RFL_DEV_MOTOR_DAMIAO_MOTOR == 1 */

    default:
        break;
    }
}

/**
 * @brief 电机执行控制
 */
void rflMotorExecuteControl(rfl_motor_s *motor)
{
    switch (motor->type)
    {
#if (RFL_DEV_MOTOR_RM_MOTOR == 1)
    case RFL_MOTOR_RM_M2006:
    case RFL_MOTOR_RM_M3508:
    case RFL_MOTOR_RM_GM6020:
        break;
#endif /* RFL_DEV_MOTOR_RM_MOTOR == 1 */

#if (RFL_DEV_MOTOR_UNITREE_MOTOR == 1)
    case RFL_MOTOR_UNITREE_GO_M8010_6:

        unitree_motor_control((unitree_motor_s *)(motor->driver));

        break;
#endif /* RFL_DEV_MOTOR_RM_MOTOR == 1 */

#if (RFL_DEV_MOTOR_DAMIAO_MOTOR == 1)

    case RFL_MOTOR_DM_J8009_2EC:

        if (motor->mode_ == RFL_MOTOR_CONTROL_MODE_NO_FORCE && motor->last_mode != RFL_MOTOR_CONTROL_MODE_NO_FORCE)
        {
            damiao_motor_enable((damiao_motor_s *)(motor->driver), false);
            motor->last_mode = RFL_MOTOR_CONTROL_MODE_NO_FORCE;
        }
        else if (motor->mode_ != RFL_MOTOR_CONTROL_MODE_NO_FORCE && motor->last_mode == RFL_MOTOR_CONTROL_MODE_NO_FORCE)
        {
            damiao_motor_enable((damiao_motor_s *)(motor->driver), true);
            motor->last_mode = motor->mode_;
        }

        switch (motor->mode_)
        {
        case RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE:
            damiao_motor_pos_speed_control((damiao_motor_s *)(motor->driver), motor->set_angle_.rad, motor->max_speed_);
            break;

        default:
            break;
        }

        break;
#endif /* RFL_DEV_MOTOR_DAMIAO_MOTOR == 1 */

    default:
        break;
    }
}

/**
 * @brief 重置电机角度
 */
void rflMotorResetAngle(rfl_motor_s *motor, rfl_angle_format_e angle_format, float source_angle,
                        bool security_restriction)
{
    rfl_angle_s angle = {0};
    rflAngleUpdate(&angle, angle_format, source_angle * (motor->is_reversed ? -1.0f : 1.0f)); // 输入输出需考虑反转

    if (security_restriction)
    {
        if (angle.deg > motor->max_angle_.deg)
            rflAngleUpdate(&angle, RFL_ANGLE_FORMAT_DEGREE, motor->max_angle_.deg);
        else if (angle.deg < motor->min_angle_.deg)
            rflAngleUpdate(&angle, RFL_ANGLE_FORMAT_DEGREE, motor->min_angle_.deg);
    }

    switch (motor->type)
    {
#if (RFL_DEV_MOTOR_RM_MOTOR == 1)
    case RFL_MOTOR_RM_M2006:
    case RFL_MOTOR_RM_M3508:
    case RFL_MOTOR_RM_GM6020:
        rm_motor_reset_angle((rm_motor_s *)(motor->driver), angle.deg);
        break;
#endif /* RFL_DEV_MOTOR_RM_MOTOR == 1 */

#if (RFL_DEV_MOTOR_UNITREE_MOTOR == 1)
    case RFL_MOTOR_UNITREE_GO_M8010_6:
        unitree_motor_reset_angle((unitree_motor_s *)(motor->driver), angle.rad);
        break;
#endif /* RFL_DEV_MOTOR_UNITREE_MOTOR == 1 */

    default:
        break;
    }

    rflAngleUpdate(&motor->set_angle_, RFL_ANGLE_FORMAT_DEGREE, angle.deg);

    if (motor->controller_type == RFL_MOTOR_CONTROLLER_PID)
    {
        PID_clear(&((rfl_motor_pid_controller_s *)(motor->controller))->angle_pid);
        PID_clear(&((rfl_motor_pid_controller_s *)(motor->controller))->speed_pid);
    }
}

/**
 * @brief 设置电机控制模式
 */
void rflMotorSetMode(rfl_motor_s *motor, rfl_motor_control_mode_e mode)
{
    if (motor->controller_type == RFL_MOTOR_CONTROLLER_PID)
    {
        if (mode == RFL_MOTOR_CONTROL_MODE_NO_FORCE)
        {
            motor->mode_ = mode;
            PID_clear(&((rfl_motor_pid_controller_s *)(motor->controller))->angle_pid);
            PID_clear(&((rfl_motor_pid_controller_s *)(motor->controller))->speed_pid);
            motor->control_output_ = motor->set_speed_ = motor->max_speed_ = 0.0f;
            rflAngleUpdate(&motor->track_angle, RFL_ANGLE_FORMAT_DEGREE, motor->angle_.deg);
            rflAngleUpdate(&motor->set_angle_, RFL_ANGLE_FORMAT_DEGREE, motor->angle_.deg);
        }

        if (motor->mode_ == RFL_MOTOR_CONTROL_MODE_NO_FORCE && mode == RFL_MOTOR_CONTROL_MODE_SPEED)
        {
            motor->mode_ = mode;
            rflAngleUpdate(&motor->track_angle, RFL_ANGLE_FORMAT_DEGREE, 0.0f);
            rflAngleUpdate(&motor->set_angle_, RFL_ANGLE_FORMAT_DEGREE, 0.0f);
        }

        if (motor->mode_ == RFL_MOTOR_CONTROL_MODE_NO_FORCE && mode == RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE)
        {
            motor->mode_ = mode;
        }
        else if (motor->mode_ == RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE && mode == RFL_MOTOR_CONTROL_MODE_ANGLE)
        {
            motor->mode_ = mode;
            rflAngleUpdate(&motor->track_angle, RFL_ANGLE_FORMAT_DEGREE, 0.0f);
        }
        else if (motor->mode_ == RFL_MOTOR_CONTROL_MODE_ANGLE && mode == RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE)
        {
            motor->mode_ = mode;
        }

        if (motor->mode_ == RFL_MOTOR_CONTROL_MODE_NO_FORCE && mode == RFL_MOTOR_CONTROL_MODE_DIRECTION)
        {
            motor->mode_ = mode;
        }

        if (motor->mode_ == RFL_MOTOR_CONTROL_MODE_ANGLE)
            motor->angle_format = RFL_MOTOR_ANGLE_FORMAT_CIRCLED;
        else if (motor->mode_ == RFL_MOTOR_CONTROL_MODE_DIRECTION)
            motor->angle_format = RFL_MOTOR_ANGLE_FORMAT_ABSOLUTE;
    }
    else if (motor->controller_type == RFL_MOTOR_CONTROLLER_UNITREE)
    {
        if (mode == RFL_MOTOR_CONTROL_MODE_NO_FORCE)
        {
            motor->mode_ = mode;
            rflAngleUpdate(&motor->track_angle, RFL_ANGLE_FORMAT_DEGREE, motor->angle_.deg);
            rflAngleUpdate(&motor->set_angle_, RFL_ANGLE_FORMAT_DEGREE, motor->angle_.deg);
        }

        if (motor->mode_ == RFL_MOTOR_CONTROL_MODE_NO_FORCE && mode == RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE)
        {
            motor->mode_ = mode;
        }
        else if (motor->mode_ == RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE && mode == RFL_MOTOR_CONTROL_MODE_ANGLE)
        {
            motor->mode_ = mode;
            rflAngleUpdate(&motor->track_angle, RFL_ANGLE_FORMAT_DEGREE, 0.0f);
        }
        else if (motor->mode_ == RFL_MOTOR_CONTROL_MODE_ANGLE && mode == RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE)
        {
            motor->mode_ = mode;
        }
    }
    else if (motor->controller_type == RFL_MOTOR_CONTROLLER_DAMIAO)
    {
        motor->mode_ = mode;
    }
}
/**
 * @brief 设置电机预期角速度
 */
void rflMotorSetSpeed(rfl_motor_s *motor, float set_speed)
{
    motor->set_speed_ = set_speed;
}
/**
 * @brief 设置电机最大角速度
 */
void rflMotorSetMaxSpeed(rfl_motor_s *motor, float max_speed)
{
    if (motor->mode_ == RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE)
    {
        motor->max_speed_ = max_speed;
    }
}
/**
 * @brief 设置电机预期角度
 */
void rflMotorSetAngle(rfl_motor_s *motor, rfl_angle_format_e angle_format, float angle)
{
    rflAngleUpdate(&motor->set_angle_, angle_format, angle);
}
/**
 * @brief 设置电机角度范围
 */
void rflMotorSetDegAngleLimit(rfl_motor_s *motor, rfl_angle_format_e angle_format, float max_angle, float min_angle)
{
    rflAngleUpdate(&motor->max_angle_, angle_format, max_angle);
    rflAngleUpdate(&motor->min_angle_, angle_format, min_angle);
}

/**
 * @brief 获取电机当前模式
 */
rfl_motor_control_mode_e rflMotorGetMode(rfl_motor_s *motor)
{
    return motor->mode_;
}
/**
 * @brief 获取电机当前转矩
 */
float rflMotorGetTorque(rfl_motor_s *motor)
{
    return motor->torque_;
}
/**
 * @brief 获取电机最大可达角度
 */
float rflMotorGetMaxAngle(rfl_motor_s *motor, rfl_angle_format_e angle_format)
{
    if (angle_format == RFL_ANGLE_FORMAT_DEGREE)
        return motor->max_angle_.deg;
    else if (angle_format == RFL_ANGLE_FORMAT_RADIAN)
        return motor->max_angle_.rad;

    return 0.0f;
}
/**
 * @brief 获取电机最小可达角度
 */
float rflMotorGetMinAngle(rfl_motor_s *motor, rfl_angle_format_e angle_format)
{
    if (angle_format == RFL_ANGLE_FORMAT_DEGREE)
        return motor->min_angle_.deg;
    else if (angle_format == RFL_ANGLE_FORMAT_RADIAN)
        return motor->min_angle_.rad;

    return 0.0f;
}
/**
 * @brief 获取电机当前速度 单位 rad * s^-1
 */
float rflMotorGetSpeed(rfl_motor_s *motor)
{
    return motor->speed_;
}
/**
 * @brief 获取电机当前角度
 */
float rflMotorGetAngle(rfl_motor_s *motor, rfl_angle_format_e angle_format)
{
    if (angle_format == RFL_ANGLE_FORMAT_DEGREE)
        return motor->angle_.deg;
    else if (angle_format == RFL_ANGLE_FORMAT_RADIAN)
        return motor->angle_.rad;

    return 0.0f;
}
/**
 * @brief 获取电机当前输出
 */
float rflMotorGetOutput(rfl_motor_s *motor)
{
    return motor->control_output_;
}
