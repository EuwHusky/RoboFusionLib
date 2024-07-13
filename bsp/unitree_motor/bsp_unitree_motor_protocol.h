#ifndef _BSP_UNITREE_MOTOR_PROTOCOL_H__
#define _BSP_UNITREE_MOTOR_PROTOCOL_H__

#include "dev_motor_config.h"

#if RFL_BSP_UNITREE_MOTOR_ENABLED

#include "stdbool.h"
#include "stdint.h"

#pragma pack(1)

/**
 * @brief 电机模式控制信息
 *
 */
typedef struct
{
    uint8_t id : 4;     // 电机ID: 0,1...,13,14 15表示向所有电机广播数据(此时无返回)
    uint8_t status : 3; // 工作模式: 0.锁定 1.FOC闭环 2.编码器校准 3.保留
    uint8_t none : 1;   // 保留位
} RIS_Mode_t;           // 控制模式 1Byte

/**
 * @brief 电机状态控制信息
 *
 */
typedef struct
{
    int16_t tor_des; // 期望关节输出扭矩 unit: N.m      (q8)
    int16_t spd_des; // 期望关节输出速度 unit: rad/s    (q8)
    int32_t pos_des; // 期望关节输出位置 unit: rad      (q15)
    int16_t k_pos;   // 期望关节刚度系数 unit: -1.0-1.0 (q15)
    int16_t k_spd;   // 期望关节阻尼系数 unit: -1.0-1.0 (q15)

} RIS_Comd_t; // 控制参数 12Byte

/**
 * @brief 电机状态反馈信息
 *
 */
typedef struct
{
    int16_t torque;      // 实际关节输出扭矩 unit: N.m     (q8)
    int16_t speed;       // 实际关节输出速度 unit: rad/s   (q8)
    int32_t pos;         // 实际关节输出位置 unit: rad     (q15)
    int8_t temp;         // 电机温度: -128~127°C
    uint8_t MError : 3;  // 电机错误标识: 0.正常 1.过热 2.过流 3.过压 4.编码器故障 5-7.保留
    uint16_t force : 12; // 足端气压传感器数据 12bit (0-4095)
    uint8_t none : 1;    // 保留位
} RIS_Fbk_t;             // 状态数据 11Byte

#pragma pack()

#pragma pack(1)

typedef struct UnitreeMotorFeedback
{
    uint8_t head[2];        // 包头         2Byte
    RIS_Mode_t mode;        // 电机控制模式  1Byte
    RIS_Fbk_t fbk;          // 电机反馈数据 11Byte
    uint16_t CRC16;         // CRC          2Byte
} unitree_motor_feedback_s; // 返回数据

typedef struct UnitreeMotorCommand
{
    uint8_t head[2];       // 包头         2Byte
    RIS_Mode_t mode;       // 电机控制模式  1Byte
    RIS_Comd_t comd;       // 电机期望数据 12Byte
    uint16_t CRC16;        // CRC          2Byte
} unitree_motor_command_s; // 电机控制命令数据包

#pragma pack()

#endif /* RFL_BSP_UNITREE_MOTOR_ENABLED */

#endif /* _BSP_UNITREE_MOTOR_PROTOCOL_H__ */
