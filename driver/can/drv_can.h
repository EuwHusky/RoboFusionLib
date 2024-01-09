#ifndef _DRV_CAN_H__
#define _DRV_CAN_H__

#include "stdint.h"

#include "main.h"

#include "rfl_config.h"

typedef struct RflCanRxMessage
{
    uint32_t can_id;
    uint8_t data[8];
} rfl_can_rx_msg_s;

typedef struct RflCan
{
    rfl_can_rx_msg_s *rx_msg_box;
    uint32_t *id_table;
    uint32_t box_size;
} rfl_can_rx_msg_box_s;

extern void rflCanInit(void);
extern void rflCanSendData(uint8_t can_ordinal, uint32_t can_id, uint8_t tx_data[8]);
extern uint8_t *rflCanGetRxMessageBoxData(uint8_t can_ordinal, uint32_t can_id);

/* 使用RM官方电机 */
#if (RFL_DEV_MOTOR_RM_MOTOR == 1)

extern void rflRmMotorControl(uint8_t can_ordinal, uint32_t can_id, int16_t motor1, int16_t motor2, int16_t motor3,
                              int16_t motor4);

#endif /* RFL_DEV_MOTOR_RM_MOTOR == 1 */

#endif /* _DRV_CAN_H__ */
