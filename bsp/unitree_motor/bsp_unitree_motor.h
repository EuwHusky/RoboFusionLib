#ifndef _BSP_UNITREE_MOTOR_H__
#define _BSP_UNITREE_MOTOR_H__

#include "dev_motor_config.h"

#if (RFL_DEV_MOTOR_UNITREE_MOTOR == 1)

#include "stdint.h"

#include "bsp_unitree_motor_protocol.h"

/**
 * @brief 宇树 GO-M8010-6 减速箱减速比
 */
#define UNITREE_GO_M8010_6_REDUCTION_RATIO (6.33f)

typedef struct UnitreeMotor
{
    uint16_t targer_id; // 电机ID 0~14
    uint16_t set_mode;  // 0:空闲, 1:闭环FOC控制, 2:电机标定

    float set_torque; // 期望关节的输出力矩（电机本身的力矩）(Nm)
    float set_speed;  // 期望关节速度（电机本身的速度）(rad/s)
    float set_angle;  // 期望关节位置 (rad)
    float k_a;        // 关节刚度系数
    float k_s;        // 关节速度系数

    unitree_motor_command_s command; // 电机控制数据结构体

    const unitree_motor_feedback_s *feedback; // 电机反馈数据结构体

    uint8_t is_data_correct; // 接收数据是否完整（1完整，0不完整）

    uint8_t id;         // 电机ID 0~14
    uint8_t mode;       // 0:空闲, 1:闭环FOC控制, 2:电机标定
    float torque;       // 当前实际电机输出力矩 NM
    float speed;        // speed rad/s
    float angle;        // 当前电机位置（主控0点修正，电机关节还是以编码器0点为准） rad
    int32_t temp;       // 温度
    uint8_t error_code; // 错误码
    float foot_force;   // 足端气压传感器数据 12bit (0-4095)

    // float set_shaft_speed;
    float set_shaft_angle;
    float shaft_speed;
    float shaft_angle;

    float effector_transmission_ratio;
    float angle_offset;
} unitree_motor_s;

extern void unitree_motor_init(unitree_motor_s *unitree_motor);
extern void unitree_motor_update_status(unitree_motor_s *unitree_motor);
extern void unitree_motor_control(unitree_motor_s *unitree_motor);
extern void unitree_motor_reset_angle(unitree_motor_s *unitree_motor, float rad_angle);

extern void unitree_uart_init(void);
extern const unitree_motor_feedback_s *unitree_motor_get_feedback_pointer(uint16_t unitree_motor_id);

#endif /* RFL_DEV_MOTOR_UNITREE_MOTOR == 1 */

#endif /* _BSP_UNITREE_MOTOR_H__ */
