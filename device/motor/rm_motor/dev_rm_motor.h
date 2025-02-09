#ifndef _DEV_RM_MOTOR_H__
#define _DEV_RM_MOTOR_H__

#include "dev_base_motor.h"

#include "dev_rm_motor_config.h"

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
 * @brief RM M2006 力矩转换系数（这是一个错误值，量纲单位非N*M，不可用于定量分析）
 */
#define RM_M2006_TORQUE_FACTOR (0.001f)

/**
 * @brief RM M3508 力矩转换系数
 */
#define RM_M3508_TORQUE_FACTOR (0.0003662109375f)

/**
 * @brief RM GM6020 力矩转换系数（这是一个错误值，量纲单位非N*M，不可用于定量分析）
 */
#define RM_GM6020_TORQUE_FACTOR (0.001f)

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

/* 解析RM电机反馈数据 */
#define DecodeRmMotorFeedback(ptr, data)                                                                               \
    {                                                                                                                  \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);                                                           \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);                                                     \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);                                                 \
        (ptr)->temperate = (data)[6];                                                                                  \
    }

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
} RflRmMotorFeedback;

typedef struct
{
    RflBaseMotor base;

    float ecd_to_effector_angle_factor_;
    float rpm_to_effector_speed_factor_;
    float torque_factor_;

    int16_t max_rotor_turns_;
    int16_t min_rotor_turns_;
    uint16_t last_ecd_;
    int16_t rotor_turns_;
    int16_t ecd_angle_offset_;
    int32_t ecd_angle_;

    uint8_t *can_rx_data_;
    RflRmMotorFeedback feedback_;

    float control_output_;

    RflResult (*GetControlOutput)(void *);

} RflRmMotor;

extern RflResult RflRmMotorGetDefaultConfig(RflRmMotorConfig *config, RflMotorType type,
                                            RflMotorControllerType controller_type);
extern RflResult RflRmMotorInit(RflRmMotor *self, RflRmMotorConfig *config);

#endif /* _DEV_RM_MOTOR_H__ */
