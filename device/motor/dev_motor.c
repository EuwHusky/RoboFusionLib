#include "math.h"
#include "string.h"

#include "dev_motor.h"

#include "drv_can.h"

#include "algo_data_limiting.h"

/**
 * @brief 获取电机默认配置
 */
void rflMotorGetDefaultConfig(rfl_motor_config_s *motor_config, rfl_motor_type_e type,
                              rfl_motor_controller_type_e controller)
{
    motor_config->type = type;
    motor_config->controller_type = controller;
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

#if (RFL_DEV_MOTOR_UNITREE_MOTOR == 1)
    case RFL_MOTOR_UNITREE_GO_M8010_6:

        motor_config->effector_transmission_ratio = UNITREE_GO_M8010_6_REDUCTION_RATIO;

        break;
#endif /* RFL_DEV_MOTOR_UNITREE_MOTOR == 1 */

    default:
        break;
    }

    motor_config->max_accle = 10.0f;

    rflAngleUpdate(&motor_config->max_angle, RFL_ANGLE_FORMAT_DEGREE, RFL_MOTOR_DEFAULT_ANGLE_RANGE);
    rflAngleUpdate(&motor_config->min_angle, RFL_ANGLE_FORMAT_DEGREE, -RFL_MOTOR_DEFAULT_ANGLE_RANGE);

    // 控制器参数
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
    else if (motor_config->controller_type == RFL_MOTOR_CONTROLLER_UNITREE)
    {
        switch (motor_config->type)
        {
#if (RFL_DEV_MOTOR_UNITREE_MOTOR == 1)
        case RFL_MOTOR_UNITREE_GO_M8010_6:
            motor_config->unitree_k_a = RFL_MOTOR_UNITREE_GO_M8010_6_K_ANGLE;
            motor_config->unitree_k_s = RFL_MOTOR_UNITREE_GO_M8010_6_K_SPEED;
            break;
#endif /* RFL_DEV_MOTOR_UNITREE_MOTOR == 1 */

        default:
            motor_config->unitree_k_a = RFL_MOTOR_UNITREE_DEFAULT_K_ANGLE;
            motor_config->unitree_k_s = RFL_MOTOR_UNITREE_DEFAULT_K_SPEED;
            break;
        }
    }

    motor_config->external_speed = NULL;
    motor_config->external_angle = NULL;

#if (RFL_DEV_MOTOR_RM_MOTOR == 1)
    motor_config->can_handle_id = 1;
    motor_config->can_id = 0x201;
#endif /* RFL_DEV_MOTOR_RM_MOTOR == 1 */

#if (RFL_DEV_MOTOR_UNITREE_MOTOR == 1)
    motor_config->unitree_motor_id = 0;
#endif /* RFL_DEV_MOTOR_UNITREE_MOTOR == 1 */
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

    motor->controller_type = motor_config->controller_type;
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

        ((rm_motor_s *)(motor->driver))->effector_transmission_ratio = motor->effector_transmission_ratio_;
        ((rm_motor_s *)(motor->driver))->ecd_to_effector_angle_factor =
            RM_MOTOR_ECD_TO_EFFECTOR_ANGLE_FACTOR / motor->effector_transmission_ratio_;
        ((rm_motor_s *)(motor->driver))->rpm_to_effector_speed_factor =
            RM_MOTOR_RPM_TO_EFFECTOR_SPEED_FACTOR / motor->effector_transmission_ratio_;

        rm_motor_rotor_turns_range =
            RM_MOTOR_ROTOR_TURNS_RANGE_PARAM / ((rm_motor_s *)(motor->driver))->ecd_to_effector_angle_factor;
        ((rm_motor_s *)(motor->driver))->max_rotor_turns = rm_motor_rotor_turns_range - 2;
        ((rm_motor_s *)(motor->driver))->min_rotor_turns = -(rm_motor_rotor_turns_range - 1);

        if (motor_config->can_handle_id == 1)
        {
            ((rm_motor_s *)(motor->driver))->can_rx_data = rflCan1AddRxMessageBox(motor_config->can_id);
        }
        else if (motor_config->can_handle_id == 2)
        {
            ((rm_motor_s *)(motor->driver))->can_rx_data = rflCan2AddRxMessageBox(motor_config->can_id);
        }

        rm_motor_init((rm_motor_s *)(motor->driver));

        break;
#endif /* RFL_DEV_MOTOR_RM_MOTOR == 1 */

#if (RFL_DEV_MOTOR_UNITREE_MOTOR == 1)
    case RFL_MOTOR_UNITREE_GO_M8010_6:

        motor->driver = (unitree_motor_s *)malloc(sizeof(unitree_motor_s));
        memset(motor->driver, 0, sizeof(unitree_motor_s));

        ((unitree_motor_s *)(motor->driver))->targer_id = motor_config->unitree_motor_id;

        ((unitree_motor_s *)(motor->driver))->feedback =
            unitree_motor_get_feedback_pointer(motor_config->unitree_motor_id);

        unitree_motor_init((unitree_motor_s *)(motor->driver));

        break;
#endif /* RFL_DEV_MOTOR_UNITREE_MOTOR == 1 */

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

        if (motor->external_speed == NULL)
            motor->speed = ((rm_motor_s *)(motor->driver))->speed;
        else
            motor->speed = *motor->external_speed;

        if (motor->external_angle == NULL)
            rflAngleUpdate(&motor->angle, RFL_ANGLE_FORMAT_DEGREE, ((rm_motor_s *)(motor->driver))->deg_angle);
        else
            rflAngleUpdate(&motor->angle, RFL_ANGLE_FORMAT_DEGREE, motor->external_angle->deg);

        break;
#endif /* RFL_DEV_MOTOR_RM_MOTOR == 1 */

#if (RFL_DEV_MOTOR_UNITREE_MOTOR == 1)
    case RFL_MOTOR_UNITREE_GO_M8010_6:

        unitree_motor_update_status((unitree_motor_s *)(motor->driver));

        if (motor->external_speed == NULL)
            motor->speed = ((unitree_motor_s *)(motor->driver))->speed / motor->effector_transmission_ratio_;
        else
            motor->speed = *motor->external_speed;

        if (motor->external_angle == NULL)
            rflAngleUpdate(
                &motor->angle, RFL_ANGLE_FORMAT_RADIAN,
                (((unitree_motor_s *)(motor->driver))->angle - ((unitree_motor_s *)(motor->driver))->angle_offset) /
                    motor->effector_transmission_ratio_);
        else
            rflAngleUpdate(&motor->angle, RFL_ANGLE_FORMAT_DEGREE, motor->external_angle->deg);

        break;
#endif /* RFL_DEV_MOTOR_UNITREE_MOTOR == 1 */

    default:
        break;
    }
}

/**
 * @brief 更新电机控制量
 */
float rflMotorUpdataControl(rfl_motor_s *motor)
{
    if (motor->controller_type == RFL_MOTOR_CONTROLLER_PID)
    {
        if (motor->mode_ == RFL_MOTOR_CONTROL_MODE_NO_FORCE)
        {
            PID_clear(&((rfl_motor_pid_controller_s *)(motor->controller))->angle_pid);
            PID_clear(&((rfl_motor_pid_controller_s *)(motor->controller))->speed_pid);
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
            motor->set_speed_ = PID_calc(&((rfl_motor_pid_controller_s *)(motor->controller))->angle_pid,
                                         motor->angle.deg, motor->set_angle_.deg);
        }
        else if (motor->mode_ == RFL_MOTOR_CONTROL_MODE_DIRECTION)
        {
            /* 角度限幅 */
            rflAngleUpdate(&motor->set_angle_, RFL_ANGLE_FORMAT_DEGREE,
                           rflFloatLoopConstrain(motor->set_angle_.deg, -DEG_PI, DEG_PI));

            /* 角度环PID计算 */
            motor->set_speed_ =
                PID_calc(&((rfl_motor_pid_controller_s *)(motor->controller))->angle_pid, 0.0f,
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
            motor->control_output = PID_calc(&((rfl_motor_pid_controller_s *)(motor->controller))->speed_pid,
                                             motor->speed, motor->set_speed_);

            return motor->control_output;
        }
    }
    else if (motor->controller_type == RFL_MOTOR_CONTROLLER_UNITREE)
    {
        if (motor->mode_ == RFL_MOTOR_CONTROL_MODE_NO_FORCE)
        {
            ((unitree_motor_s *)(motor->driver))->set_mode = 0;

            return 0.0f;
        }
        else if (motor->mode_ == RFL_MOTOR_CONTROL_MODE_ANGLE)
        {
            ((unitree_motor_s *)(motor->driver))->set_mode = 1;
        }

        /* 角度限幅 */
        if (motor->set_angle_.deg > motor->max_angle_.deg)
            rflAngleUpdate(&motor->set_angle_, RFL_ANGLE_FORMAT_DEGREE, motor->max_angle_.deg);
        else if (motor->set_angle_.deg < motor->min_angle_.deg)
            rflAngleUpdate(&motor->set_angle_, RFL_ANGLE_FORMAT_DEGREE, motor->min_angle_.deg);

        ((unitree_motor_s *)(motor->driver))->k_a = ((rfl_motor_unitree_controller_s *)(motor->controller))->k_angle;
        ((unitree_motor_s *)(motor->driver))->k_s = ((rfl_motor_unitree_controller_s *)(motor->controller))->k_speed;
    }

    return 0.0f;
}

/**
 * @brief 电机执行控制
 */
void rflMotorExecuteControl(rfl_motor_s *motor)
{
    switch (motor->type)
    {
#if (RFL_DEV_MOTOR_UNITREE_MOTOR == 1)
    case RFL_MOTOR_UNITREE_GO_M8010_6:

        ((unitree_motor_s *)(motor->driver))->set_angle = motor->set_angle_.rad * motor->effector_transmission_ratio_ +
                                                          ((unitree_motor_s *)(motor->driver))->angle_offset;

        unitree_motor_control((unitree_motor_s *)(motor->driver));

        break;
#endif /* RFL_DEV_MOTOR_RM_MOTOR == 1 */

    default:
        break;
    }
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
    if (motor->controller_type == RFL_MOTOR_CONTROLLER_PID)
    {
        motor->mode_ = mode;

        if (motor->mode_ == RFL_MOTOR_CONTROL_MODE_ANGLE)
            motor->angle_format = RFL_MOTOR_ANGLE_FORMAT_CIRCLED;
        else if (motor->mode_ == RFL_MOTOR_CONTROL_MODE_DIRECTION)
            motor->angle_format = RFL_MOTOR_ANGLE_FORMAT_ABSOLUTE;

        PID_clear(&((rfl_motor_pid_controller_s *)(motor->controller))->angle_pid);
        PID_clear(&((rfl_motor_pid_controller_s *)(motor->controller))->speed_pid);
        motor->control_output = motor->set_speed_ = 0.0f;
        rflAngleUpdate(&motor->set_angle_, RFL_ANGLE_FORMAT_DEGREE, 0.0f);
    }
    else if (motor->controller_type == RFL_MOTOR_CONTROLLER_UNITREE)
    {
        if (mode == RFL_MOTOR_CONTROL_MODE_NO_FORCE && mode == RFL_MOTOR_CONTROL_MODE_ANGLE)
            motor->mode_ = mode;
    }
}
/**
 * @brief 设置电机预期角速度
 */
void rflMotorSetSpeed(rfl_motor_s *motor, float set_speed)
{
    if (motor->controller_type == RFL_MOTOR_CONTROLLER_PID)
    {
        motor->set_speed_ = set_speed;
    }
}
/**
 * @brief 设置电机预期角加速度
 */
void rflMotorSetAccle(rfl_motor_s *motor, float set_accle)
{
    if (motor->controller_type == RFL_MOTOR_CONTROLLER_PID)
    {
        motor->max_accle_ = set_accle;
    }
}
/**
 * @brief 设置电机预期角度-角度值
 */
void rflMotorSetDegAngle(rfl_motor_s *motor, float degree_angle)
{
    rflAngleUpdate(&motor->set_angle_, RFL_ANGLE_FORMAT_DEGREE, degree_angle);
}
/**
 * @brief 设置电机预期角度-弧度制
 */
void rflMotorSetRadAngle(rfl_motor_s *motor, float radian_angle)
{
    rflAngleUpdate(&motor->set_angle_, RFL_ANGLE_FORMAT_RADIAN, radian_angle);
}
/**
 * @brief 设置电机角度范围-角度值
 */
void rflMotorSetDegAngleLimit(rfl_motor_s *motor, float max_degree_angle, float min_degree_angle)
{
    rflAngleUpdate(&motor->max_angle_, RFL_ANGLE_FORMAT_DEGREE, max_degree_angle);
    rflAngleUpdate(&motor->min_angle_, RFL_ANGLE_FORMAT_DEGREE, min_degree_angle);
}
/**
 * @brief 设置电机角度范围-弧度制
 */
void rflMotorSetRadAngleLimit(rfl_motor_s *motor, float max_radian_angle, float min_radian_angle)
{
    rflAngleUpdate(&motor->max_angle_, RFL_ANGLE_FORMAT_RADIAN, max_radian_angle);
    rflAngleUpdate(&motor->min_angle_, RFL_ANGLE_FORMAT_RADIAN, min_radian_angle);
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
        rm_motor_reset_angle((rm_motor_s *)(motor->driver));
        break;
#endif /* RFL_DEV_MOTOR_RM_MOTOR == 1 */

    default:
        break;
    }
}
