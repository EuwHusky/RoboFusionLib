/**
 * @file kine_stable_chassis.c
 * @author EHusky
 * @brief 机器人运动学模块库底盘模块
 *
 * @copyright Copyright (c) 2024 CNU W.PIE
 *
 */

#include "math.h"
#include "stdlib.h"
#include "string.h"

#include "kine_stable_chassis.h"
#include "kine_stable_chassis_controller.h"
#include "kine_stable_chassis_parameter.h"

#include "algo_data_limiting.h"
#include "algo_math.h"

/**
 * @brief 更新底盘状态量
 *
 * @param chassis 底盘实体结构体指针
 */
static void chassis_update_status(rfl_chassis_s *chassis);
/**
 * @brief 更新底盘角速度控制量
 *
 * @param chassis 底盘实体结构体指针
 */
static void chassis_update_wz_set(rfl_chassis_s *chassis);
/**
 * @brief 更新底盘电机控制输出值
 *
 * @param chassis 底盘实体结构体指针
 */
static void chassis_update_motor_control(rfl_chassis_s *chassis);

/**
 * @brief 获取底盘默认配置
 *
 * @param config 底盘配置结构体指针
 * @param type 底盘类型
 */
void rflChassisGetDefaultConfig(rfl_chassis_config_s *config, rfl_chassis_type_e type)
{
    memset(config, 0, sizeof(rfl_chassis_config_s));

    config->type = type;
    config->mode = RFL_CHASSIS_BEHAVIOR_NO_FORCE;
    switch (config->type)
    {
    case RFL_CHASSIS_MECANUM:
        config->mecanum_length = RFL_CHASSIS_MECANUM_DEFAULT_LENGTH;
        config->mecanum_width = RFL_CHASSIS_MECANUM_DEFAULT_WIDTH;
        break;
    case RFL_CHASSIS_OMNI:
        config->length = RFL_CHASSIS_OMNI_DEFAULT_LENGTH;
        break;
    case RFL_CHASSIS_DUAL_STEER:
        /*默认舵轮位于左前和右后*/
        config->steer_motor_orientation = 0;
        config->length = RFL_CHASSIS_STEER_DEFAULT_LENGTH;
        break;
    case RFL_CHASSIS_FOUR_STEER:
        config->length = RFL_CHASSIS_STEER_DEFAULT_LENGTH;
        break;
    default:
        break;
    }

    config->reference_frame = RFL_CHASSIS_INERTIAL_FRAME;

    config->set_control_vector = NULL;
    config->direction_controller_type = RFL_CHASSIS_CONTROLLER_PID;
    config->direction_pid_param[0] = RFL_CHASSIS_DEFAULT_DIRECTION_PID_KP;
    config->direction_pid_param[1] = RFL_CHASSIS_DEFAULT_DIRECTION_PID_KI;
    config->direction_pid_param[2] = RFL_CHASSIS_DEFAULT_DIRECTION_PID_KD;
    config->direction_pid_param[3] = RFL_CHASSIS_DEFAULT_DIRECTION_PID_MAX_IOUT;
    config->direction_pid_param[4] = RFL_CHASSIS_DEFAULT_DIRECTION_PID_MAX_OUT;
}

/**
 * @brief 初始化底盘实体
 *
 * @param chassis 底盘实体结构体指针
 * @param config 底盘配置结构体指针
 * @param forward_vector 参考坐标系下底盘结构方向
 * @param motor_feedback 底盘电机组反馈量
 */
void rflChassisInit(rfl_chassis_s *chassis, rfl_chassis_config_s *config, const rfl_angle_s *forward_vector,
                    const float *motor_feedback)
{
    memset(chassis, 0, sizeof(rfl_chassis_s));

    chassis->type = config->type;
    chassis->behavior = config->mode;

    switch (config->type)
    {
    case RFL_CHASSIS_MECANUM:
        chassis->parameter = (rfl_chassis_mecanum_parameter_s *)malloc(sizeof(rfl_chassis_mecanum_parameter_s));
        ((rfl_chassis_mecanum_parameter_s *)chassis->parameter)->length = config->mecanum_length;
        ((rfl_chassis_mecanum_parameter_s *)chassis->parameter)->width = config->mecanum_width;
        break;
    case RFL_CHASSIS_OMNI:
        chassis->parameter = (rfl_chassis_omni_parameter_s *)malloc(sizeof(rfl_chassis_omni_parameter_s));
        ((rfl_chassis_omni_parameter_s *)chassis->parameter)->length = config->length;
        break;
    case RFL_CHASSIS_DUAL_STEER:
        chassis->parameter = (rfl_chassis_dual_steer_parameter_s *)malloc(sizeof(rfl_chassis_dual_steer_parameter_s));
        ((rfl_chassis_dual_steer_parameter_s *)chassis->parameter)->steer_motor_orientation =
            config->steer_motor_orientation;
        ((rfl_chassis_dual_steer_parameter_s *)chassis->parameter)->length = config->length;
        break;
    case RFL_CHASSIS_FOUR_STEER:
        chassis->parameter = (rfl_chassis_four_steer_parameter_s *)malloc(sizeof(rfl_chassis_four_steer_parameter_s));
        ((rfl_chassis_dual_steer_parameter_s *)chassis->parameter)->steer_motor_orientation =
            config->steer_motor_orientation;
        ((rfl_chassis_four_steer_parameter_s *)chassis->parameter)->length = config->length;
        break;
    default:
        chassis->parameter = NULL;
        break;
    }

    chassis->reference_frame = config->reference_frame;
    chassis->forward_vector = forward_vector;
    rflAngleUpdate(&chassis->set_forward_vector, RFL_ANGLE_FORMAT_DEGREE, chassis->forward_vector->deg);

    if (chassis->reference_frame == RFL_CHASSIS_INERTIAL_FRAME && config->set_control_vector != NULL)
    {
        chassis->set_control_vector = config->set_control_vector;
        rflAngleUpdate(&chassis->control_vector, RFL_ANGLE_FORMAT_DEGREE, chassis->set_control_vector->deg);
    }
    else
    {
        chassis->set_control_vector = NULL;
        rflAngleUpdate(&chassis->control_vector, RFL_ANGLE_FORMAT_DEGREE, 0.0f);
    }
    rflAngleUpdate(&chassis->follow_offset, RFL_ANGLE_FORMAT_DEGREE, 0.0f);

    if (config->direction_controller_type == RFL_CHASSIS_CONTROLLER_PID)
    {
        chassis->direction_controller =
            (rfl_chassis_normal_pid_controller_s *)malloc(sizeof(rfl_chassis_normal_pid_controller_s));
        memset(chassis->direction_controller, 0, sizeof(rfl_chassis_normal_pid_controller_s));

        const float direction_pid_param[3] = {config->direction_pid_param[0], config->direction_pid_param[1],
                                              config->direction_pid_param[2]};
        PID_init(&((rfl_chassis_normal_pid_controller_s *)(chassis->direction_controller))->angle_pid, PID_POSITION,
                 direction_pid_param, config->direction_pid_param[4], config->direction_pid_param[3]);
    }
    else if (config->direction_controller_type == RFL_CHASSIS_CONTROLLER_FEEDFORWARD_PID)
    {
    }

    switch (config->type)
    {
    case RFL_CHASSIS_MECANUM:
    case RFL_CHASSIS_OMNI:
    case RFL_CHASSIS_DUAL_STEER:
        chassis->motor_num = 4; // 四个电机
        break;
    case RFL_CHASSIS_FOUR_STEER:
        chassis->motor_num = 8; // 八个电机
        break;
    default:
        chassis->motor_num = 4;
        break;
    }

    chassis->motor_feedback = motor_feedback;
    chassis->motor_output = (float *)malloc(chassis->motor_num * sizeof(float));

    if (config->type == RFL_CHASSIS_DUAL_STEER || config->type == RFL_CHASSIS_FOUR_STEER)
        chassis->reverse_motor_output = (bool *)malloc(chassis->motor_num / 2 * sizeof(bool));
    else
        chassis->reverse_motor_output = NULL;
}

/**
 * @brief 更新底盘实体
 *
 * @param chassis 底盘实体结构体指针
 */
void rflChassisUpdate(rfl_chassis_s *chassis)
{
    if (chassis->reference_frame == RFL_CHASSIS_INERTIAL_FRAME && chassis->set_control_vector != NULL)
        rflAngleUpdate(&chassis->control_vector, RFL_ANGLE_FORMAT_DEGREE,
                       rflFloatLoopConstrain(chassis->set_control_vector->deg, -DEG_PI, DEG_PI));
    else
        rflAngleUpdate(&chassis->control_vector, RFL_ANGLE_FORMAT_DEGREE, 0.0f);

    chassis_update_status(chassis);

    chassis_update_wz_set(chassis);

    chassis_update_motor_control(chassis);
}

/**
 * @brief 设定底盘行为模式
 *
 * @param chassis 底盘实体结构体指针
 * @param mode 要设定的行为模式
 */
void rflChassisSetBehavior(rfl_chassis_s *chassis, rfl_chassis_behavior_e mode)
{
    // 机体系下FREEZE模式等价于FOLLOW模式
    if (chassis->reference_frame == RFL_CHASSIS_CONTROL_FRAME && mode == RFL_CHASSIS_BEHAVIOR_FREEZE)
        return;

    chassis->behavior = mode;
}

/**
 * @brief 设定跟随模式下结构正方向相对于控制正方向的偏角
 *
 * @param chassis 底盘实体结构体指针
 * @param angle_format 偏角的角度制
 * @param angle_value 偏角的角度值
 */
void rflChassisSetFollowOffset(rfl_chassis_s *chassis, rfl_angle_format_e angle_format, float angle_value)
{
    rflAngleUpdate(&chassis->follow_offset, angle_format, angle_value);
}

/**
 * @brief 设定底盘速度
 *
 * @param chassis 底盘实体结构体指针
 * @param vx 要设定的前进后退速度
 * @param vy 要设定的左移右移速度
 * @param wz 要设定的小陀螺自转速度
 */
void rflChassisSetSpeedVector(rfl_chassis_s *chassis, float vx, float vy, float wz)
{
    chassis->set_vx = vx;
    chassis->set_vy = vy;
    if (chassis->behavior == RFL_CHASSIS_BEHAVIOR_SPIN)
        chassis->set_wz = wz;
}

/**
 * @brief 获取底盘当前模式
 *
 * @param chassis 底盘实体结构体指针
 * @return float 底盘当前行为模式
 */
rfl_chassis_behavior_e rflChassisGetBehavior(rfl_chassis_s *chassis)
{
    return chassis->behavior;
}

/**
 * @brief 获取底盘速度向量
 *
 * @param chassis 底盘实体结构体指针
 * @return float* 在底盘控制坐标系下的底盘速度向量
 */
float *rflChassisGetSpeedVector(rfl_chassis_s *chassis)
{
    return chassis->speed_vector;
}

/**
 * @brief 获取底盘电机组输出值
 *
 * @param chassis 底盘实体结构体指针
 * @return float* 底盘电机组输出值
 */
float *rflChassisGetMotorOutputArray(rfl_chassis_s *chassis)
{
    return chassis->motor_output;
}

/**
 * @brief 更新底盘状态量
 *
 * @param chassis 底盘实体结构体指针
 */
static void chassis_update_status(rfl_chassis_s *chassis)
{
    float body_frame_vx = 0.0f;
    float body_frame_vy = 0.0f;
    float body_frame_wz = 0.0f;

    if (chassis->type == RFL_CHASSIS_MECANUM)
    {
        body_frame_vx = (chassis->motor_feedback[0] + chassis->motor_feedback[1] + chassis->motor_feedback[2] +
                         chassis->motor_feedback[3]) /
                        4.0f;
        body_frame_vy = (-chassis->motor_feedback[0] + chassis->motor_feedback[1] - chassis->motor_feedback[2] +
                         chassis->motor_feedback[3]) /
                        4.0f;
        body_frame_wz = (-chassis->motor_feedback[0] - chassis->motor_feedback[1] + chassis->motor_feedback[2] +
                         chassis->motor_feedback[3]) /
                        ((((rfl_chassis_mecanum_parameter_s *)chassis->parameter)->length +
                          ((rfl_chassis_mecanum_parameter_s *)chassis->parameter)->width) *
                         2.0f);
    }
    else if (chassis->type == RFL_CHASSIS_OMNI)
    {
        float calculating_factor = 0.1767766952f; // 此值为 √2 / 8
        body_frame_vx = (chassis->motor_feedback[0] + chassis->motor_feedback[1] + chassis->motor_feedback[2] +
                         chassis->motor_feedback[3]) *
                        calculating_factor;
        body_frame_vy = (-chassis->motor_feedback[0] + chassis->motor_feedback[1] - chassis->motor_feedback[2] +
                         chassis->motor_feedback[3]) *
                        calculating_factor;
        body_frame_wz = (-chassis->motor_feedback[0] - chassis->motor_feedback[1] + chassis->motor_feedback[2] +
                         chassis->motor_feedback[3]) /
                        4.0f;
    }
    else if (chassis->type == RFL_CHASSIS_DUAL_STEER)
    {
        body_frame_vx = (cosf(chassis->motor_feedback[2]) * chassis->motor_feedback[0] +
                         cosf(chassis->motor_feedback[3]) * chassis->motor_feedback[1]) /
                        2.0f;
        body_frame_vy = (sinf(chassis->motor_feedback[2]) * chassis->motor_feedback[0] +
                         sinf(chassis->motor_feedback[3]) * chassis->motor_feedback[1]) /
                        2.0f;
        if (((rfl_chassis_dual_steer_parameter_s *)chassis->parameter)->steer_motor_orientation == 0)
        {
            body_frame_wz = (cosf(rflFloatLoopConstrain(chassis->motor_feedback[2] -
                                                            2.3561944901923449288469825374596f, // 自旋系偏角为+135度
                                                        -RAD_PI, RAD_PI) *
                                  chassis->motor_feedback[0]) +
                             cosf(rflFloatLoopConstrain(chassis->motor_feedback[3] +
                                                            0.78539816339744830961566084581988f, // 自旋系偏角为-45度
                                                        -RAD_PI, RAD_PI) *
                                  chassis->motor_feedback[1])) /
                            2.0f;
        }
        else if (((rfl_chassis_dual_steer_parameter_s *)chassis->parameter)->steer_motor_orientation == 0)
        {
            body_frame_wz = (cosf(rflFloatLoopConstrain(chassis->motor_feedback[2] +
                                                            2.3561944901923449288469825374596f, // 自旋系偏角为-135度
                                                        -RAD_PI, RAD_PI) *
                                  chassis->motor_feedback[0]) +
                             cosf(rflFloatLoopConstrain(chassis->motor_feedback[3] -
                                                            0.78539816339744830961566084581988f, // 自旋系偏角为+45度
                                                        -RAD_PI, RAD_PI) *
                                  chassis->motor_feedback[1])) /
                            2.0f;
        }
    }
    else if (chassis->type == RFL_CHASSIS_FOUR_STEER)
    {
        body_frame_vx = (cosf(chassis->motor_feedback[4]) * chassis->motor_feedback[0] +
                         cosf(chassis->motor_feedback[5]) * chassis->motor_feedback[1] +
                         cosf(chassis->motor_feedback[6]) * chassis->motor_feedback[2] +
                         cosf(chassis->motor_feedback[7]) * chassis->motor_feedback[3]) /
                        4.0f;
        body_frame_vy = (sinf(chassis->motor_feedback[4]) * chassis->motor_feedback[0] +
                         sinf(chassis->motor_feedback[5]) * chassis->motor_feedback[1] +
                         sinf(chassis->motor_feedback[6]) * chassis->motor_feedback[2] +
                         sinf(chassis->motor_feedback[7]) * chassis->motor_feedback[3]) /
                        4.0f;
        body_frame_wz = ((cosf(rflFloatLoopConstrain(chassis->motor_feedback[4] -
                                                         2.3561944901923449288469825374596f, // 自旋系偏角为+135度
                                                     -RAD_PI, RAD_PI)) *
                          chassis->motor_feedback[0]) +
                         (cosf(rflFloatLoopConstrain(chassis->motor_feedback[5] +
                                                         2.3561944901923449288469825374596f, // 自旋系偏角为-135度
                                                     -RAD_PI, RAD_PI)) *
                          chassis->motor_feedback[1]) +
                         (cosf(rflFloatLoopConstrain(chassis->motor_feedback[6] +
                                                         0.78539816339744830961566084581988f, // 自旋系偏角为-45度
                                                     -RAD_PI, RAD_PI)) *
                          chassis->motor_feedback[2]) +
                         (cosf(rflFloatLoopConstrain(chassis->motor_feedback[7] -
                                                         0.78539816339744830961566084581988f, // 自旋系偏角为+45度
                                                     -RAD_PI, RAD_PI)) *
                          chassis->motor_feedback[3])) /
                        4.0f;
    }

    float cos_theta = cosf(chassis->control_vector.rad - chassis->forward_vector->rad);
    float sin_theta = sinf(chassis->control_vector.rad - chassis->forward_vector->rad);

    chassis->speed_vector[0] = cos_theta * body_frame_vx + sin_theta * body_frame_vy;
    chassis->speed_vector[1] = -sin_theta * body_frame_vx + cos_theta * body_frame_vy;
    chassis->speed_vector[2] = body_frame_wz;
}

/**
 * @brief 更新底盘角速度控制量
 *
 * @param chassis 底盘实体结构体指针
 */
static void chassis_update_wz_set(rfl_chassis_s *chassis)
{
    switch (chassis->behavior)
    {
    case RFL_CHASSIS_BEHAVIOR_NO_FORCE:
        rflAngleUpdate(&chassis->set_forward_vector, RFL_ANGLE_FORMAT_DEGREE, chassis->forward_vector->deg);
        PID_clear(&((rfl_chassis_normal_pid_controller_s *)chassis->direction_controller)->angle_pid);
        chassis->set_wz = 0.0f;
        break;
    case RFL_CHASSIS_BEHAVIOR_FOLLOW_CONTROL:
        rflAngleUpdate(
            &chassis->set_forward_vector, RFL_ANGLE_FORMAT_DEGREE,
            rflFloatLoopConstrain(chassis->control_vector.deg + chassis->follow_offset.deg, -DEG_PI, DEG_PI));
        chassis->set_wz = PID_calc(
            &((rfl_chassis_normal_pid_controller_s *)(chassis->direction_controller))->angle_pid, 0.0f,
            rflFloatLoopConstrain(chassis->set_forward_vector.deg - chassis->forward_vector->deg, -DEG_PI, DEG_PI));
        break;
    case RFL_CHASSIS_BEHAVIOR_FREEZE:
        // chassis->set_wz = PID_calc(
        //     &((rfl_chassis_normal_pid_controller_s *)(chassis->direction_controller))->angle_pid, 0.0f,
        //     rflFloatLoopConstrain(chassis->set_forward_vector.deg - chassis->forward_vector->deg, -DEG_PI, DEG_PI));
        chassis->set_wz = 0.0f;
        break;
    case RFL_CHASSIS_BEHAVIOR_SPIN:
        rflAngleUpdate(&chassis->set_forward_vector, RFL_ANGLE_FORMAT_DEGREE, chassis->forward_vector->deg);
        // 角速度由用户设定
        break;
    default:
        rflAngleUpdate(&chassis->set_forward_vector, RFL_ANGLE_FORMAT_DEGREE, chassis->forward_vector->deg);
        PID_clear(&((rfl_chassis_normal_pid_controller_s *)chassis->direction_controller)->angle_pid);
        chassis->set_wz = 0.0f;
        break;
    }
}

/**
 * @brief 更新底盘电机控制输出值
 *
 * @param chassis 底盘实体结构体指针
 */
static void chassis_update_motor_control(rfl_chassis_s *chassis)
{
    if (chassis->behavior == RFL_CHASSIS_BEHAVIOR_NO_FORCE)
    {
        if (chassis->type == RFL_CHASSIS_DUAL_STEER || chassis->type == RFL_CHASSIS_FOUR_STEER)
        {
            for (uint8_t i = 0; i < chassis->motor_num / 2; i++)
            {
                // 驱动电机速度置0
                chassis->motor_output[i] = 0.0f;

                // 舵向电机角度不变
                chassis->motor_output[i + chassis->motor_num / 2] = chassis->motor_feedback[i + chassis->motor_num / 2];
            }
        }
        else
        {
            for (uint8_t i = 0; i < 4; i++)
            {
                chassis->motor_output[i] = 0.0f;
            }
        }
    }
    else
    {
        float cos_theta = cosf(chassis->forward_vector->rad - chassis->control_vector.rad);
        float sin_theta = sinf(chassis->forward_vector->rad - chassis->control_vector.rad);

        float body_frame_set_vx = cos_theta * chassis->set_vx + sin_theta * chassis->set_vy;
        float body_frame_set_vy = -sin_theta * chassis->set_vx + cos_theta * chassis->set_vy;

        if (chassis->type == RFL_CHASSIS_MECANUM)
        {
            float wheel_speed_by_wz = chassis->set_wz *
                                      (((rfl_chassis_mecanum_parameter_s *)chassis->parameter)->length +
                                       ((rfl_chassis_mecanum_parameter_s *)chassis->parameter)->width) *
                                      0.5f;
            chassis->motor_output[0] = body_frame_set_vx - body_frame_set_vy - wheel_speed_by_wz;
            chassis->motor_output[1] = body_frame_set_vx + body_frame_set_vy - wheel_speed_by_wz;
            chassis->motor_output[2] = body_frame_set_vx - body_frame_set_vy + wheel_speed_by_wz;
            chassis->motor_output[3] = body_frame_set_vx + body_frame_set_vy + wheel_speed_by_wz;
        }
        else if (chassis->type == RFL_CHASSIS_OMNI)
        {
            float wheel_speed_by_vx = body_frame_set_vx * 1.41421356237309504f; // 乘√2
            float wheel_speed_by_vy = body_frame_set_vy * 1.41421356237309504f; // 乘√2
            float wheel_speed_by_wz = chassis->set_wz * ((rfl_chassis_omni_parameter_s *)chassis->parameter)->length;
            chassis->motor_output[0] = wheel_speed_by_vx - wheel_speed_by_vy - wheel_speed_by_wz;
            chassis->motor_output[1] = wheel_speed_by_vx + wheel_speed_by_vy - wheel_speed_by_wz;
            chassis->motor_output[2] = wheel_speed_by_vx - wheel_speed_by_vy + wheel_speed_by_wz;
            chassis->motor_output[3] = wheel_speed_by_vx + wheel_speed_by_vy + wheel_speed_by_wz;
        }
        else if (chassis->type == RFL_CHASSIS_DUAL_STEER)
        {
            // 和四舵一样 只用删掉部分电机 暂时懒得做了
        }
        else if (chassis->type == RFL_CHASSIS_FOUR_STEER)
        {
            float wheel_speed_by_wz =
                chassis->set_wz * ((rfl_chassis_four_steer_parameter_s *)chassis->parameter)->length;

            /* 将自旋速度分解到XY平动速度上
             * 神秘系数分别是四个舵轮的自旋速度分量正方向在结构坐标系的角度的cos值和sin值
             * cos值用于分解到X轴 sin值用于分解到Y轴
             * 自旋速度分量分别为135度、-135度、-45度、45度 */
            float mixed_vx[4] = {0.0f};
            mixed_vx[0] = body_frame_set_vx - 0.70710677f * wheel_speed_by_wz;
            mixed_vx[1] = body_frame_set_vx - 0.70710677f * wheel_speed_by_wz;
            mixed_vx[2] = body_frame_set_vx + 0.70710677f * wheel_speed_by_wz;
            mixed_vx[3] = body_frame_set_vx + 0.70710677f * wheel_speed_by_wz;
            float mixed_vy[4] = {0.0f};
            mixed_vy[0] = body_frame_set_vy + 0.70710677f * wheel_speed_by_wz;
            mixed_vy[1] = body_frame_set_vy - 0.70710677f * wheel_speed_by_wz;
            mixed_vy[2] = body_frame_set_vy - 0.70710677f * wheel_speed_by_wz;
            mixed_vy[3] = body_frame_set_vy + 0.70710677f * wheel_speed_by_wz;

            for (uint8_t i = 0; i < 4; i++)
            {
                /*勾股定理求轮速*/
                chassis->motor_output[i] = rflSqrt(mixed_vx[i] * mixed_vx[i] + mixed_vy[i] * mixed_vy[i]);

                /*四象限反正切求舵向角度*/
                chassis->motor_output[i + 4] = atan2f(mixed_vy[i], mixed_vx[i]);
            }

            /* 舵向电机转角劣化 当舵向单次转角超过45度时 通过反转轮子速度方向的方式使得所需转角重新小于45度 */
            for (uint8_t i = 0; i < 4; i++)
            {
                if (fabsf(rflFloatLoopConstrain(chassis->motor_feedback[i + 4] - chassis->motor_output[i + 4], -RAD_PI,
                                                RAD_PI)) > 1.5707964f)
                {
                    chassis->motor_output[i + 4] =
                        rflFloatLoopConstrain(chassis->motor_output[i + 4] + RAD_PI, -RAD_PI, RAD_PI);
                    chassis->reverse_motor_output[i] = true;
                }
                else
                {
                    chassis->reverse_motor_output[i] = false;
                }

                chassis->motor_output[i] *= (chassis->reverse_motor_output[i] ? -1.0f : 1.0f);
            }

            /*底盘静止时舵向姿态*/
            if ((fabs(body_frame_set_vx) + fabs(body_frame_set_vy) + fabs(chassis->set_wz)) <= 1e-6)
            {
                if (chassis->behavior == RFL_CHASSIS_BEHAVIOR_FOLLOW_CONTROL)
                {
                    /*舵向姿态为自旋姿态 锁定底盘平动*/
                    chassis->motor_output[4] = -0.7853982f;
                    chassis->motor_output[5] = 0.7853982f;
                    chassis->motor_output[6] = -0.7853982f;
                    chassis->motor_output[7] = 0.7853982f;
                }
                else if (chassis->behavior == RFL_CHASSIS_BEHAVIOR_FREEZE)
                {
                    /*舵向姿态跟随控制系 快速响应前后移动*/
                    chassis->motor_output[4] = chassis->control_vector.rad;
                    chassis->motor_output[5] = chassis->control_vector.rad;
                    chassis->motor_output[6] = chassis->control_vector.rad;
                    chassis->motor_output[7] = chassis->control_vector.rad;
                }
            }
        }
    }
}
