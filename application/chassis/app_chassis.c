#include "stdlib.h"
#include "string.h"

#include "app_chassis.h"
#include "app_chassis_controller.h"

#include "bsp_rm_motor.h"

#include "algo_sign.h"

static void chassis_update_status(rfl_chassis_s *chassis);
static void chassis_update_control(rfl_chassis_s *chassis);

void rflChassisGetDefaultConfig(rfl_chassis_config_s *chassis_config, rfl_chassis_type_e type)
{
    memset(chassis_config, 0, sizeof(rfl_chassis_config_s));

    chassis_config->type = RFL_CHASSIS_MECANUM;
    chassis_config->mode = RFL_CHASSIS_CONTROL_MODE_FOLLOW;

    memset(chassis_config->mecanum_polarity, 0, 4 * sizeof(uint8_t));
    memset(chassis_config->motor_polarity, 0, 8 * sizeof(uint8_t));
    memset(chassis_config->is_steer_motor, 0, 8 * sizeof(bool));
    chassis_config->wheel_radius = 0.0f;
    chassis_config->length = 0.0f;
    chassis_config->width = 0.0f;
    chassis_config->inner_radius = 0.0f;
    chassis_config->outer_radius = 0.0f;
    switch (chassis_config->type)
    {
    case RFL_CHASSIS_MECANUM:
        // 默认电机参数
        chassis_config->wheel_radius = RFL_CHASSIS_DEFAULT_MECANUM_WHEEL_RADIUS;
        chassis_config->length = RFL_CHASSIS_MECANUM_DEFAULT_LENGTH;
        chassis_config->width = RFL_CHASSIS_MECANUM_DEFAULT_WIDTH;
        break;
    case RFL_CHASSIS_COAXIAL_MECANUM:
        // 默认麦克纳姆轮极性
        chassis_config->mecanum_polarity[0] = 0;
        chassis_config->mecanum_polarity[1] = 1;
        chassis_config->mecanum_polarity[2] = 0;
        chassis_config->mecanum_polarity[3] = 1;
        // 默认电机参数
        chassis_config->wheel_radius = RFL_CHASSIS_DEFAULT_MECANUM_WHEEL_RADIUS;
        chassis_config->inner_radius = RFL_CHASSIS_COAXIAL_MECANUM_DEFAULT_INNER_RADIUS;
        chassis_config->outer_radius = RFL_CHASSIS_COAXIAL_MECANUM_DEFAULT_OUTER_RADIUS;
        break;
    case RFL_CHASSIS_OMNI:
        // 默认电机参数
        chassis_config->wheel_radius = RFL_CHASSIS_DEFAULT_DRIVING_WHEEL_RADIUS;
        chassis_config->length = RFL_CHASSIS_STEER_DEFAULT_LENGTH;
        break;
    case RFL_CHASSIS_DUAL_STEER:
        // 默认电机顺序
        for (uint8_t i = 0; i < CHASSIS_DUAL_STEER_MOTOR_NUM; i++)
        {
            if (i % 2 == 0)
                chassis_config->is_steer_motor[i] = true;
            else
                chassis_config->is_steer_motor[i] = false;
        }
        // 默认电机参数
        chassis_config->wheel_radius = RFL_CHASSIS_DEFAULT_DRIVING_WHEEL_RADIUS;
        chassis_config->length = RFL_CHASSIS_STEER_DEFAULT_LENGTH;
        break;
    case RFL_CHASSIS_FOUR_STEER:
        // 默认电机顺序
        for (uint8_t i = 0; i < CHASSIS_FOUR_STEER_MOTOR_NUM; i++)
        {
            if (i % 2 == 0)
                chassis_config->is_steer_motor[i] = true;
            else
                chassis_config->is_steer_motor[i] = false;
        }
        // 默认电机参数
        chassis_config->wheel_radius = RFL_CHASSIS_DEFAULT_DRIVING_WHEEL_RADIUS;
        chassis_config->length = RFL_CHASSIS_STEER_DEFAULT_LENGTH;
        break;
    default:
        break;
    }

    chassis_config->controller_type = RFL_CHASSIS_CONTROLLER_PID;

    chassis_config->angle_pid_param[0] = RFL_CHASSIS_DEFAULT_ANGLE_PID_KP;
    chassis_config->angle_pid_param[1] = RFL_CHASSIS_DEFAULT_ANGLE_PID_KI;
    chassis_config->angle_pid_param[2] = RFL_CHASSIS_DEFAULT_ANGLE_PID_KD;
    chassis_config->angle_pid_param[3] = RFL_CHASSIS_DEFAULT_ANGLE_PID_MAX_IOUT;
    chassis_config->angle_pid_param[4] = RFL_CHASSIS_DEFAULT_ANGLE_PID_MAX_OUT;

    rflAngleUpdate(&chassis_config->angle_offset, RFL_ANGLE_FORMAT_DEGREE, 0.0f);
    chassis_config->external_angle = NULL;

    switch (chassis_config->type)
    {
    case RFL_CHASSIS_MECANUM:
    case RFL_CHASSIS_COAXIAL_MECANUM:
    case RFL_CHASSIS_OMNI:
        chassis_config->motor_num = 4; // 四个电机
        chassis_config->motor_config_list =
            (rfl_chassis_motor_config_s *)malloc(chassis_config->motor_num * sizeof(rfl_chassis_motor_config_s));
        for (uint8_t i = 0; i < chassis_config->motor_num; i++)
        {
            chassis_config->motor_config_list[i].motor_type = RFL_MOTOR_RM_M3508;
            chassis_config->motor_config_list[i].effector_transmission_ratio = RM_M3508_REDUCTION_RATIO;

            chassis_config->motor_config_list[i].external_angle = NULL;

            chassis_config->motor_config_list[i].speed_pid_param[0] = RFL_CHASSIS_DEFAULT_DRIVING_RM_M3508_SPEED_PID_KP;
            chassis_config->motor_config_list[i].speed_pid_param[1] = RFL_CHASSIS_DEFAULT_DRIVING_RM_M3508_SPEED_PID_KI;
            chassis_config->motor_config_list[i].speed_pid_param[2] = RFL_CHASSIS_DEFAULT_DRIVING_RM_M3508_SPEED_PID_KD;
            chassis_config->motor_config_list[i].speed_pid_param[3] =
                RFL_CHASSIS_DEFAULT_DRIVING_RM_M3508_SPEED_PID_MAX_IOUT;
            chassis_config->motor_config_list[i].speed_pid_param[4] =
                RFL_CHASSIS_DEFAULT_DRIVING_RM_M3508_SPEED_PID_MAX_OUT;

            chassis_config->motor_config_list[i].can_ordinal = 1;
            chassis_config->motor_config_list[i].can_id = 0x201 + i;
        }
        break;
    case RFL_CHASSIS_DUAL_STEER:
        chassis_config->motor_num = 4; // 四个电机
        chassis_config->motor_config_list =
            (rfl_chassis_motor_config_s *)malloc(chassis_config->motor_num * sizeof(rfl_chassis_motor_config_s));
        for (uint8_t i = 0; i < chassis_config->motor_num; i++)
        {
            chassis_config->motor_config_list[i].motor_type = RFL_MOTOR_RM_M3508;
            chassis_config->motor_config_list[i].external_angle = NULL;
            if (chassis_config->is_steer_motor[i] == true) // 舵向电机
            {
                chassis_config->motor_config_list[i].effector_transmission_ratio = 1;

                chassis_config->motor_config_list[i].speed_pid_param[0] =
                    RFL_CHASSIS_DEFAULT_STEER_RM_M3508_SPEED_PID_KP;
                chassis_config->motor_config_list[i].speed_pid_param[1] =
                    RFL_CHASSIS_DEFAULT_STEER_RM_M3508_SPEED_PID_KI;
                chassis_config->motor_config_list[i].speed_pid_param[2] =
                    RFL_CHASSIS_DEFAULT_STEER_RM_M3508_SPEED_PID_KD;
                chassis_config->motor_config_list[i].speed_pid_param[3] =
                    RFL_CHASSIS_DEFAULT_STEER_RM_M3508_SPEED_PID_MAX_IOUT;
                chassis_config->motor_config_list[i].speed_pid_param[4] =
                    RFL_CHASSIS_DEFAULT_STEER_RM_M3508_SPEED_PID_MAX_OUT;

                chassis_config->motor_config_list[i].angle_pid_param[0] =
                    RFL_CHASSIS_DEFAULT_STEER_RM_M3508_ANGLE_PID_KP;
                chassis_config->motor_config_list[i].angle_pid_param[1] =
                    RFL_CHASSIS_DEFAULT_STEER_RM_M3508_ANGLE_PID_KI;
                chassis_config->motor_config_list[i].angle_pid_param[2] =
                    RFL_CHASSIS_DEFAULT_STEER_RM_M3508_ANGLE_PID_KD;
                chassis_config->motor_config_list[i].angle_pid_param[3] =
                    RFL_CHASSIS_DEFAULT_STEER_RM_M3508_ANGLE_PID_MAX_IOUT;
                chassis_config->motor_config_list[i].angle_pid_param[4] =
                    RFL_CHASSIS_DEFAULT_STEER_RM_M3508_ANGLE_PID_MAX_OUT;
            }
            else // 驱动电机
            {
                chassis_config->motor_config_list[i].effector_transmission_ratio = RM_M3508_REDUCTION_RATIO;

                chassis_config->motor_config_list[i].speed_pid_param[0] =
                    RFL_CHASSIS_DEFAULT_DRIVING_RM_M3508_SPEED_PID_KP;
                chassis_config->motor_config_list[i].speed_pid_param[1] =
                    RFL_CHASSIS_DEFAULT_DRIVING_RM_M3508_SPEED_PID_KI;
                chassis_config->motor_config_list[i].speed_pid_param[2] =
                    RFL_CHASSIS_DEFAULT_DRIVING_RM_M3508_SPEED_PID_KD;
                chassis_config->motor_config_list[i].speed_pid_param[3] =
                    RFL_CHASSIS_DEFAULT_DRIVING_RM_M3508_SPEED_PID_MAX_IOUT;
                chassis_config->motor_config_list[i].speed_pid_param[4] =
                    RFL_CHASSIS_DEFAULT_DRIVING_RM_M3508_SPEED_PID_MAX_OUT;
            }
            chassis_config->motor_config_list[i].can_ordinal = 1;
            chassis_config->motor_config_list[i].can_id = 0x201 + i;
        }
    case RFL_CHASSIS_FOUR_STEER:
        chassis_config->motor_num = 8; // 八个电机
        chassis_config->motor_config_list = (rfl_chassis_motor_config_s *)malloc(
            chassis_config->motor_num * sizeof(rfl_chassis_motor_config_s)); // 八个电机
        for (uint8_t i = 0; i < chassis_config->motor_num; i++)
        {
            chassis_config->motor_config_list[i].motor_type = RFL_MOTOR_RM_M3508;
            chassis_config->motor_config_list[i].external_angle = NULL;
            if (chassis_config->is_steer_motor[i] == true) // 舵向电机
            {
                chassis_config->motor_config_list[i].effector_transmission_ratio = 1;

                chassis_config->motor_config_list[i].speed_pid_param[0] =
                    RFL_CHASSIS_DEFAULT_STEER_RM_M3508_SPEED_PID_KP;
                chassis_config->motor_config_list[i].speed_pid_param[1] =
                    RFL_CHASSIS_DEFAULT_STEER_RM_M3508_SPEED_PID_KI;
                chassis_config->motor_config_list[i].speed_pid_param[2] =
                    RFL_CHASSIS_DEFAULT_STEER_RM_M3508_SPEED_PID_KD;
                chassis_config->motor_config_list[i].speed_pid_param[3] =
                    RFL_CHASSIS_DEFAULT_STEER_RM_M3508_SPEED_PID_MAX_IOUT;
                chassis_config->motor_config_list[i].speed_pid_param[4] =
                    RFL_CHASSIS_DEFAULT_STEER_RM_M3508_SPEED_PID_MAX_OUT;

                chassis_config->motor_config_list[i].angle_pid_param[0] =
                    RFL_CHASSIS_DEFAULT_STEER_RM_M3508_ANGLE_PID_KP;
                chassis_config->motor_config_list[i].angle_pid_param[1] =
                    RFL_CHASSIS_DEFAULT_STEER_RM_M3508_ANGLE_PID_KI;
                chassis_config->motor_config_list[i].angle_pid_param[2] =
                    RFL_CHASSIS_DEFAULT_STEER_RM_M3508_ANGLE_PID_KD;
                chassis_config->motor_config_list[i].angle_pid_param[3] =
                    RFL_CHASSIS_DEFAULT_STEER_RM_M3508_ANGLE_PID_MAX_IOUT;
                chassis_config->motor_config_list[i].angle_pid_param[4] =
                    RFL_CHASSIS_DEFAULT_STEER_RM_M3508_ANGLE_PID_MAX_OUT;

                chassis_config->motor_config_list[i].can_ordinal = 2;
                chassis_config->motor_config_list[i].can_id = 0x201 + i;
            }
            else // 驱动电机
            {
                chassis_config->motor_config_list[i].effector_transmission_ratio = RM_M3508_REDUCTION_RATIO;

                chassis_config->motor_config_list[i].speed_pid_param[0] =
                    RFL_CHASSIS_DEFAULT_DRIVING_RM_M3508_SPEED_PID_KP;
                chassis_config->motor_config_list[i].speed_pid_param[1] =
                    RFL_CHASSIS_DEFAULT_DRIVING_RM_M3508_SPEED_PID_KI;
                chassis_config->motor_config_list[i].speed_pid_param[2] =
                    RFL_CHASSIS_DEFAULT_DRIVING_RM_M3508_SPEED_PID_KD;
                chassis_config->motor_config_list[i].speed_pid_param[3] =
                    RFL_CHASSIS_DEFAULT_DRIVING_RM_M3508_SPEED_PID_MAX_IOUT;
                chassis_config->motor_config_list[i].speed_pid_param[4] =
                    RFL_CHASSIS_DEFAULT_DRIVING_RM_M3508_SPEED_PID_MAX_OUT;

                chassis_config->motor_config_list[i].can_ordinal = 1;
                chassis_config->motor_config_list[i].can_id = 0x201 + i;
            }
        }
        break;
    default:
        chassis_config->motor_config_list = NULL;
        break;
    }
}

void rflChassisInit(rfl_chassis_s *chassis, rfl_chassis_config_s *chassis_config)
{
    memset(chassis, 0, sizeof(rfl_chassis_s));

    chassis->type = chassis_config->type;
    chassis->mode_ = chassis_config->mode;
    chassis->wheel_radius = chassis_config->wheel_radius;

    switch (chassis_config->type)
    {
    case RFL_CHASSIS_MECANUM:
        chassis->parameter = (rfl_chassis_mecanum_parameter_s *)malloc(sizeof(rfl_chassis_mecanum_parameter_s));
        ((rfl_chassis_mecanum_parameter_s *)chassis->parameter)->length = chassis_config->length;
        ((rfl_chassis_mecanum_parameter_s *)chassis->parameter)->width = chassis_config->width;
        break;
    case RFL_CHASSIS_COAXIAL_MECANUM:
        chassis->parameter =
            (rfl_chassis_coaxial_mecanum_parameter_s *)malloc(sizeof(rfl_chassis_coaxial_mecanum_parameter_s));
        for (uint8_t i = 0; i < chassis_config->motor_num; i++)
            ((rfl_chassis_coaxial_mecanum_parameter_s *)chassis->parameter)->mecanum_polarity[i] =
                chassis_config->mecanum_polarity[i];
        ((rfl_chassis_coaxial_mecanum_parameter_s *)chassis->parameter)->inner_radius = chassis_config->inner_radius;
        ((rfl_chassis_coaxial_mecanum_parameter_s *)chassis->parameter)->outer_radius = chassis_config->outer_radius;
        break;
    case RFL_CHASSIS_OMNI:
        chassis->parameter = (rfl_chassis_omni_parameter_s *)malloc(sizeof(rfl_chassis_omni_parameter_s));
        ((rfl_chassis_omni_parameter_s *)chassis->parameter)->length = chassis_config->length;
        break;
    case RFL_CHASSIS_DUAL_STEER:
        chassis->parameter = (rfl_chassis_dual_steer_parameter_s *)malloc(sizeof(rfl_chassis_dual_steer_parameter_s));
        for (uint8_t i = 0; i < chassis_config->motor_num; i++)
            ((rfl_chassis_dual_steer_parameter_s *)chassis->parameter)->is_steer_motor[i] =
                chassis_config->is_steer_motor[i];
        ((rfl_chassis_dual_steer_parameter_s *)chassis->parameter)->length = chassis_config->length;
        break;
    case RFL_CHASSIS_FOUR_STEER:
        chassis->parameter = (rfl_chassis_four_steer_parameter_s *)malloc(sizeof(rfl_chassis_four_steer_parameter_s));
        for (uint8_t i = 0; i < chassis_config->motor_num; i++)
            ((rfl_chassis_four_steer_parameter_s *)chassis->parameter)->is_steer_motor[i] =
                chassis_config->is_steer_motor[i];
        ((rfl_chassis_four_steer_parameter_s *)chassis->parameter)->length = chassis_config->length;
        break;
    default:
        chassis->parameter = NULL;
        break;
    }

    chassis->set_vx_ = 0.0f;
    chassis->set_vy_ = 0.0f;
    chassis->set_wz_ = 0.0f;

    if (chassis_config->controller_type == RFL_CHASSIS_CONTROLLER_PID)
    {
        chassis->controller =
            (rfl_chassis_normal_pid_controller_s *)malloc(sizeof(rfl_chassis_normal_pid_controller_s));
        memset(chassis->controller, 0, sizeof(rfl_chassis_normal_pid_controller_s));

        const float angle_pid_param[3] = {chassis_config->angle_pid_param[0], chassis_config->angle_pid_param[1],
                                          chassis_config->angle_pid_param[2]};
        PID_init(&((rfl_chassis_normal_pid_controller_s *)(chassis->controller))->angle_pid, PID_POSITION,
                 angle_pid_param, chassis_config->angle_pid_param[4], chassis_config->angle_pid_param[3]);
    }
    else if (chassis_config->controller_type == RFL_CHASSIS_CONTROLLER_FEEDFORWARD_PID)
    {
    }

    rflAngleUpdate(&chassis->angle_offset, RFL_ANGLE_FORMAT_DEGREE, chassis_config->angle_offset.deg);
    chassis->external_angle = chassis_config->external_angle;
    rflAngleUpdate(&chassis->angle_, RFL_ANGLE_FORMAT_DEGREE, 0.0f);

    chassis->motor_num = chassis_config->motor_num;
    chassis->motors = (rfl_motor_s *)malloc(chassis_config->motor_num * sizeof(rfl_motor_s));
    rfl_motor_config_s motor_config;
    for (uint8_t i = 0; i < chassis_config->motor_num; i++)
    {
        rflMotorGetDefaultConfig(&motor_config, chassis_config->motor_config_list[i].motor_type,
                                 RFL_MOTOR_CONTROLLER_PID);

        motor_config.mode = RFL_MOTOR_CONTROL_MODE_SPEED;
        motor_config.effector_transmission_ratio = chassis_config->motor_config_list[i].effector_transmission_ratio;

        motor_config.angle_pid_kp = chassis_config->motor_config_list[i].angle_pid_param[0];
        motor_config.angle_pid_ki = chassis_config->motor_config_list[i].angle_pid_param[1];
        motor_config.angle_pid_kd = chassis_config->motor_config_list[i].angle_pid_param[2];
        motor_config.angle_pid_max_iout = chassis_config->motor_config_list[i].angle_pid_param[3];
        motor_config.angle_pid_max_out = chassis_config->motor_config_list[i].angle_pid_param[4];
        motor_config.speed_pid_kp = chassis_config->motor_config_list[i].speed_pid_param[0];
        motor_config.speed_pid_ki = chassis_config->motor_config_list[i].speed_pid_param[1];
        motor_config.speed_pid_kd = chassis_config->motor_config_list[i].speed_pid_param[2];
        motor_config.speed_pid_max_iout = chassis_config->motor_config_list[i].speed_pid_param[3];
        motor_config.speed_pid_max_out = chassis_config->motor_config_list[i].speed_pid_param[4];

        motor_config.external_angle = chassis_config->motor_config_list[i].external_angle;

        motor_config.can_ordinal = chassis_config->motor_config_list[i].can_ordinal;
        motor_config.can_id = chassis_config->motor_config_list[i].can_id;

        rflMotorInit(&chassis->motors[i], &motor_config);
    }
}

void rflChassisUpdate(rfl_chassis_s *chassis)
{
    // 更新底盘状态量
    chassis_update_status(chassis);

    // 更新底盘控制量
    chassis_update_control(chassis);
}

void chassis_update_status(rfl_chassis_s *chassis)
{
    // 更新电机数据
    for (uint8_t i = 0; i < chassis->motor_num; i++)
        rflMotorUpdateStatus(&chassis->motors[i]);

    // 更新底盘状态量
    if (chassis->type == RFL_CHASSIS_MECANUM)
    {
        const uint8_t motor_polarity[4] = {1, 1, 0, 0};
        float wheels_speed[4] = {0.0f};
        for (uint8_t i = 0; i < chassis->motor_num; i++)
        {
            wheels_speed[i] = rflConvertTfToSign(motor_polarity[i]) * chassis->motors[i].speed_ * chassis->wheel_radius;
        }
        chassis->vx_ = (wheels_speed[0] + wheels_speed[1] + wheels_speed[2] + wheels_speed[3]) / 4.0f;
        chassis->vy_ = (wheels_speed[0] - wheels_speed[1] + wheels_speed[2] - wheels_speed[3]) / 4.0f;
        chassis->wz_ = (wheels_speed[0] + wheels_speed[1] - wheels_speed[2] - wheels_speed[3]) /
                       ((((rfl_chassis_mecanum_parameter_s *)chassis->parameter)->length +
                         ((rfl_chassis_mecanum_parameter_s *)chassis->parameter)->width) *
                        2.0f);
    }
    else if (chassis->type == RFL_CHASSIS_COAXIAL_MECANUM)
    {
        const uint8_t motor_polarity[4] = {1, 0, 0, 1};
        float wheels_speed[4] = {0.0f};
        for (uint8_t i = 0; i < chassis->motor_num; i++)
        {
            wheels_speed[i] = rflConvertTfToSign(motor_polarity[i]) * chassis->motors[i].speed_ * chassis->wheel_radius;
        }

        chassis->vx_ = (wheels_speed[0] + wheels_speed[1] + wheels_speed[2] + wheels_speed[3]) / 4.0f;

        float calculating_polarity[4] = {0.0f};
        for (uint8_t i = 0; i < chassis->motor_num; i++)
        {
            calculating_polarity[i] = rflConvertTfToSign(
                ((rfl_chassis_coaxial_mecanum_parameter_s *)chassis->parameter)->mecanum_polarity[0]);
        }
        chassis->vy_ = (calculating_polarity[0] * wheels_speed[0] + calculating_polarity[1] * wheels_speed[1] +
                        calculating_polarity[2] * wheels_speed[2] + calculating_polarity[3] * wheels_speed[3]) /
                       4.0f;

        chassis->wz_ = ((wheels_speed[0] - wheels_speed[2]) /
                        (4.0f * ((rfl_chassis_coaxial_mecanum_parameter_s *)chassis->parameter)->inner_radius)) +
                       ((wheels_speed[1] - wheels_speed[3]) /
                        (4.0f * ((rfl_chassis_coaxial_mecanum_parameter_s *)chassis->parameter)->outer_radius));
    }
    else if (chassis->type == RFL_CHASSIS_OMNI)
    {
        const uint8_t motor_polarity[4] = {1, 1, 0, 0};
        float wheels_speed[4] = {0.0f};
        for (uint8_t i = 0; i < chassis->motor_num; i++)
        {
            wheels_speed[i] = rflConvertTfToSign(motor_polarity[i]) * chassis->motors[i].speed_ * chassis->wheel_radius;
        }
        float factor = 0.1767766952f; // 此值为 √2 / 8

        chassis->vx_ = (wheels_speed[0] + wheels_speed[1] + wheels_speed[2] + wheels_speed[3]) * factor;
        chassis->vy_ = (wheels_speed[0] - wheels_speed[1] + wheels_speed[2] - wheels_speed[3]) * factor;
        chassis->vy_ = (wheels_speed[0] + wheels_speed[1] - wheels_speed[2] - wheels_speed[3]) / 4.0f;
    }
    else if (chassis->type == RFL_CHASSIS_DUAL_STEER)
    {
    }
    else if (chassis->type == RFL_CHASSIS_FOUR_STEER)
    {
    }
}

void chassis_update_control(rfl_chassis_s *chassis)
{
}
