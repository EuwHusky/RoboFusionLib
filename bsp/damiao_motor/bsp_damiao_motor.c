#include "string.h"

#include "bsp_damiao_motor.h"

#if (RFL_DEV_MOTOR_DAMIAO_MOTOR == 1)

#include "drv_delay.h"

#include "drv_can.h"

#include "algo_value.h"

static uint8_t data_enable[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};    // 电机使能命令
static uint8_t data_failure[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};   // 电机失能命令
static uint8_t data_save_zero[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE}; // 电机保存零点命令

static uint16_t damiao_motor_get_mode_id_set(damiao_motor_mode_e mode)
{
    return (mode == DAMIAO_MOTOR_MODE_MIT)
               ? CONTROL_MODE_ID_SET_MIT
               : ((mode == DAMIAO_MOTOR_MODE_POS_SPEED) ? CONTROL_MODE_ID_SET_POS_SPEED : CONTROL_MODE_ID_SET_SPEED);
}

static void damiao_motor_control_delay(uint16_t ms)
{
    rflOsDelayMs(ms);
}

void damiao_motor_init(damiao_motor_s *damiao_motor, damiao_motor_mode_e mode, uint8_t can, uint32_t master_id,
                       uint32_t slave_id, float p_max, float v_max, float t_max)
{
    damiao_motor->mode = mode;

    damiao_motor->can_ordinal = can;
    damiao_motor->master_can_id = master_id;
    damiao_motor->slave_can_id = slave_id;

    rflCanRxMessageBoxAddId(damiao_motor->can_ordinal, damiao_motor->master_can_id);
    damiao_motor_control_delay(5);
    damiao_motor->can_rx_data = rflCanGetRxMessageBoxData(damiao_motor->can_ordinal, damiao_motor->master_can_id);

    damiao_motor->p_max = p_max;
    damiao_motor->v_max = v_max;
    damiao_motor->t_max = t_max;
}

void damiao_motor_update_status(damiao_motor_s *damiao_motor)
{
    uint32_t feedback[3];
    feedback[0] = (damiao_motor->can_rx_data[1] << 8) | damiao_motor->can_rx_data[2];
    feedback[1] = (damiao_motor->can_rx_data[3] << 4) | (damiao_motor->can_rx_data[4] >> 4);
    feedback[2] = ((damiao_motor->can_rx_data[4] & 0xF) << 8) | damiao_motor->can_rx_data[5];
    damiao_motor->position = rflUintToFloat(feedback[0], -damiao_motor->p_max, damiao_motor->p_max, 16);
    damiao_motor->velocity = rflUintToFloat(feedback[1], -damiao_motor->v_max, damiao_motor->v_max, 12);
    damiao_motor->torque = rflUintToFloat(feedback[2], -damiao_motor->t_max, damiao_motor->t_max, 12);
}

void damiao_motor_enable(damiao_motor_s *damiao_motor, bool enable)
{
    rflCanSendData(damiao_motor->can_ordinal,
                   damiao_motor_get_mode_id_set(damiao_motor->mode) + damiao_motor->slave_can_id,
                   enable ? data_enable : data_failure);
    damiao_motor_control_delay(1);
}

void damiao_motor_pos_speed_control(damiao_motor_s *damiao_motor, float set_pos, float set_vel)
{
    uint8_t *pbuf, *vbuf;
    pbuf = (uint8_t *)&set_pos;
    vbuf = (uint8_t *)&set_vel;

    damiao_motor->can_tx_data[0] = *pbuf;
    damiao_motor->can_tx_data[1] = *(pbuf + 1);
    damiao_motor->can_tx_data[2] = *(pbuf + 2);
    damiao_motor->can_tx_data[3] = *(pbuf + 3);
    damiao_motor->can_tx_data[4] = *vbuf;
    damiao_motor->can_tx_data[5] = *(vbuf + 1);
    damiao_motor->can_tx_data[6] = *(vbuf + 2);
    damiao_motor->can_tx_data[7] = *(vbuf + 3);

    rflCanSendData(damiao_motor->can_ordinal, CONTROL_MODE_ID_SET_POS_SPEED + damiao_motor->slave_can_id,
                   damiao_motor->can_tx_data);
    damiao_motor_control_delay(1);
}

#endif /* RFL_DEV_MOTOR_DAMIAO_MOTOR == 1 */
