#include "bsp_rm_motor.h"

#include "drv_delay.h"

#include "drv_can.h"

#if (RFL_DEV_MOTOR_RM_MOTOR == 1)

#define CONTROL_MODE_NO_FORCE 0  // 无力
#define CONTROL_MODE_SPEED 1     // 速度
#define CONTROL_MODE_ANGLE 2     // 角度
#define CONTROL_MODE_DIRECTION 3 // 方位

#define ANGLE_FORMAT_CIRCLED 0  // 多圈角度
#define ANGLE_FORMAT_ABSOLUTE 1 // 绝对角度

static void rm_motor_control_delay(uint16_t ms)
{
    rflOsDelayMs(ms);
}

void rm_motor_init(rm_motor_s *rm_motor)
{
    rflCanRxMessageBoxAddId(rm_motor->can_ordinal, rm_motor->master_can_id);
    rm_motor_control_delay(5);
    rm_motor->can_rx_data = rflCanGetRxMessageBoxData(rm_motor->can_ordinal, rm_motor->master_can_id);
    rm_motor_control_delay(5);

    // 更新电机反馈数据
    getRmMotorFeedback(&rm_motor->feedback_, rm_motor->can_rx_data);

    // 记录上电时ecd
    rm_motor->ecd_angle_offset = rm_motor->feedback_.ecd;

    // 初始化上次ecd
    rm_motor->last_ecd = rm_motor->feedback_.ecd;

    rm_motor->ecd_to_effector_angle_factor =
        RM_MOTOR_ECD_TO_EFFECTOR_ANGLE_FACTOR / rm_motor->effector_transmission_ratio;
    rm_motor->rpm_to_effector_speed_factor =
        RM_MOTOR_RPM_TO_EFFECTOR_SPEED_FACTOR / rm_motor->effector_transmission_ratio;
    rm_motor->max_rotor_turns =
        (int16_t)(RM_MOTOR_ROTOR_TURNS_RANGE_PARAM / rm_motor->ecd_to_effector_angle_factor - 2);
    rm_motor->min_rotor_turns =
        -(int16_t)(RM_MOTOR_ROTOR_TURNS_RANGE_PARAM / rm_motor->ecd_to_effector_angle_factor - 1);
}

void rm_motor_update_status(rm_motor_s *rm_motor, uint8_t control_mode, uint8_t angle_format)
{
    // 更新电机反馈数据
    getRmMotorFeedback(&rm_motor->feedback_, rm_motor->can_rx_data);

    if (control_mode == CONTROL_MODE_SPEED)
    {
        rm_motor->rotor_turns = 0;
        rm_motor->ecd_angle_offset = rm_motor->feedback_.ecd;
    }
    else if (control_mode != CONTROL_MODE_SPEED)
    {
        // 跳变沿检测
        if (rm_motor->last_ecd - rm_motor->feedback_.ecd > RM_MOTOR_HALF_ECD_RANGE)
            rm_motor->rotor_turns++;
        else if (rm_motor->feedback_.ecd - rm_motor->last_ecd > RM_MOTOR_HALF_ECD_RANGE)
            rm_motor->rotor_turns--;

        // 单圈限幅
        if (angle_format == ANGLE_FORMAT_ABSOLUTE)
        {
            int16_t half_integer_transmission_ratio = (int16_t)(rm_motor->effector_transmission_ratio) / 2;
            if (rm_motor->rotor_turns > half_integer_transmission_ratio - 1 &&
                rm_motor->feedback_.ecd > rm_motor->ecd_angle_offset)
                rm_motor->rotor_turns = -half_integer_transmission_ratio;
            else if (rm_motor->rotor_turns < 1 - half_integer_transmission_ratio &&
                     rm_motor->feedback_.ecd < rm_motor->ecd_angle_offset)
                rm_motor->rotor_turns = half_integer_transmission_ratio;
        }
    }

    rm_motor->last_ecd = rm_motor->feedback_.ecd;

    // 计算末端执行器转速
    rm_motor->speed = (float)rm_motor->feedback_.speed_rpm * rm_motor->rpm_to_effector_speed_factor;

    // 可测量最大圈数限制
    if (rm_motor->rotor_turns > rm_motor->max_rotor_turns)
        rm_motor->rotor_turns = rm_motor->max_rotor_turns;
    else if (rm_motor->rotor_turns < rm_motor->min_rotor_turns)
        rm_motor->rotor_turns = rm_motor->min_rotor_turns;

    // 计算末端执行器角度 角度范围大约为 -10000°~10000°
    rm_motor->ecd_angle =
        rm_motor->rotor_turns * RM_MOTOR_ECD_RANGE + rm_motor->feedback_.ecd - rm_motor->ecd_angle_offset;
    rm_motor->deg_angle = (float)rm_motor->ecd_angle * rm_motor->ecd_to_effector_angle_factor;

    // 计算电机转矩
    rm_motor->torque = (float)rm_motor->feedback_.given_current * rm_motor->torque_factor;
}

void rm_motor_reset_angle(rm_motor_s *rm_motor, float deg_angle)
{
    // 反向计算转子圈数和偏置 为了方便所以将偏置从无符号改为了有符号整形
    int32_t ecd_angle = (int32_t)((double)deg_angle / (double)rm_motor->ecd_to_effector_angle_factor);
    rm_motor->rotor_turns = ecd_angle / RM_MOTOR_ECD_RANGE;
    rm_motor->ecd_angle_offset = rm_motor->feedback_.ecd - (ecd_angle - rm_motor->rotor_turns * RM_MOTOR_ECD_RANGE);

    // 重新计算末端执行器角度 角度范围大约为 -10000°~10000°
    rm_motor->ecd_angle =
        rm_motor->rotor_turns * RM_MOTOR_ECD_RANGE + rm_motor->feedback_.ecd - rm_motor->ecd_angle_offset;
    rm_motor->deg_angle = (float)rm_motor->ecd_angle * rm_motor->ecd_to_effector_angle_factor;
}

#endif /* RFL_DEV_MOTOR_RM_MOTOR == 1 */
