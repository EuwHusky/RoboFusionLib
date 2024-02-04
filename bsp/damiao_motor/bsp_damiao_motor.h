#ifndef _BSP_DAMIAO_MOTOR_H__
#define _BSP_DAMIAO_MOTOR_H__

#include "rfl_config.h"

#if (RFL_DEV_MOTOR_DAMIAO_MOTOR == 1)

#include "stdbool.h"
#include "stdint.h"

#define CONTROL_MODE_ID_SET_MIT 0x000
#define CONTROL_MODE_ID_SET_POS_SPEED 0x100
#define CONTROL_MODE_ID_SET_SPEED 0x200

typedef enum DamiaoMotorMode
{
    DAMIAO_MOTOR_MODE_MIT = 0,
    DAMIAO_MOTOR_MODE_POS_SPEED,
    DAMIAO_MOTOR_MODE_SPEED,
} damiao_motor_mode_e;

typedef struct DamiaoMotor
{
    damiao_motor_mode_e mode;

    uint8_t can_ordinal;
    uint32_t master_can_id;
    uint32_t slave_can_id;

    uint8_t *can_rx_data;
    uint8_t can_tx_data[8];

    float p_max;
    float v_max;
    float t_max;

    float position;
    float velocity;
    float torque;

} damiao_motor_s;

extern void damiao_motor_init(damiao_motor_s *damiao_motor, damiao_motor_mode_e mode, uint8_t can, uint32_t master_id,
                              uint32_t slave_id, float p_max, float v_max, float t_max);

extern void damiao_motor_update_status(damiao_motor_s *damiao_motor);

extern void damiao_motor_enable(damiao_motor_s *damiao_motor, bool enable);

extern void damiao_motor_pos_speed_control(damiao_motor_s *damiao_motor, float set_pos, float set_vel);

#endif /* RFL_DEV_MOTOR_DAMIAO_MOTOR == 1 */

#endif /* _BSP_DAMIAO_MOTOR_H__ */
