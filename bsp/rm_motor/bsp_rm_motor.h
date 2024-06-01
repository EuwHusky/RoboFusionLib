#ifndef _BSP_RM_MOTOR_H__
#define _BSP_RM_MOTOR_H__

#include "dev_motor_config.h"

#if (RFL_DEV_MOTOR_RM_MOTOR == 1)

#include "stdint.h"

/**
 * @brief RM M2006 减速箱减速比
 */
#define RM_M2006_REDUCTION_RATIO (36.0f)
/**
 * @brief RM M3508 减速箱减速比
 */
#define RM_M3508_REDUCTION_RATIO (19.203208556149732620320855614973f)
/**
 * @brief RM GM6020 减速箱减速比
 */
#define RM_GM6020_REDUCTION_RATIO (1.0f)

/**
 * @brief RM M2006 力矩转换系数（不确定是否正确，可能量纲单位非N*M，但能凑合用）
 */
#define RM_M2006_TORQUE_FACTOR (0.001f)

/**
 * @brief RM M3508 力矩转换系数
 */
#define RM_M3508_TORQUE_FACTOR (0.0003662109375f)

/**
 * @brief RM电机转子编码器码值范围
 */
#define RM_MOTOR_ECD_RANGE (8191)
/**
 * @brief RM电机转子编码器码值半圈范围
 */
#define RM_MOTOR_HALF_ECD_RANGE (4096)

/**
 * @brief 电机转子编码器码值转末端执行器角度值系数
 * 算式 360.0f / RM_MOTOR_ECD_RANGE / RM_MOTOR_REDUCTION_RATIO
 * @note 默认RM_MOTOR_REDUCTION_RATIO = 1
 */
#define RM_MOTOR_ECD_TO_EFFECTOR_ANGLE_FACTOR (0.04395067757294591624954218044195f)

/**
 * @brief 电机转子RPM转换末端执行器角速度系数
 * 算式 2 * PI / 60.0f / RM_MOTOR_REDUCTION_RATIO
 * @note 默认RM_MOTOR_REDUCTION_RATIO = 1
 */
#define RM_MOTOR_RPM_TO_EFFECTOR_SPEED_FACTOR (0.10471975511965977461542144610932f)

/**
 * @brief 用于计算转子转动圈数的参数
 * 算式 10000 / RM_MOTOR_ECD_RANGE
 * @note 10000是定死的，应该够大够用，以防数据溢出用或角度精度过低而设
 */
#define RM_MOTOR_ROTOR_TURNS_RANGE_PARAM (1.2208521548040532291539494567208f)

/**
 * @brief RM电机 反馈数据
 */
typedef struct RmMotorFeedback
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
} rm_motor_feedback_s;

/* 解析RM电机反馈数据 */
#define decode_rm_motor_feedback(ptr, data)                                                                            \
    {                                                                                                                  \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);                                                           \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);                                                     \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);                                                 \
        (ptr)->temperate = (data)[6];                                                                                  \
    }

/**
 * @brief RM电机
 */
typedef struct RmMotor
{
    /* 基础参数 */

    float effector_transmission_ratio; // 末端执行器转一圈时电机转子转过的圈数
    float ecd_to_effector_angle_factor;
    float rpm_to_effector_speed_factor;
    float torque_factor;

    uint8_t can_ordinal;
    uint32_t master_can_id;
    // uint32_t slave_can_id;
    const uint8_t *can_rx_data;
    rm_motor_feedback_s feedback_;

    /* 状态量 */

    int16_t ecd_angle_offset;
    uint16_t last_ecd;
    int16_t rotor_turns;
    int16_t max_rotor_turns;
    int16_t min_rotor_turns;
    int32_t ecd_angle;

    float deg_angle;     // degree 末端执行器角度
    float speed;         // rad/s 末端执行器转速
    float torque;        // N*M 电机转矩
    uint8_t temperature; // °C 电机温度

} rm_motor_s;

extern void rm_motor_init(rm_motor_s *rm_motor, bool angle_zeroed);

extern void rm_motor_update_status(rm_motor_s *rm_motor, uint8_t control_mode, uint8_t angle_format);

extern void rm_motor_reset_angle(rm_motor_s *rm_motor, float deg_angle);

#endif /* RFL_DEV_MOTOR_RM_MOTOR == 1 */

#endif /* _BSP_RM_MOTOR_H__ */
