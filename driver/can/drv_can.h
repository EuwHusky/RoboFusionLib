#ifndef _DRV_CAN_H__
#define _DRV_CAN_H__

#include "stdint.h"

#include "main.h"

#include "dev_motor_config.h"

typedef struct RflCanRxMessage
{
    uint32_t can_id;
    uint8_t rx_data[8];
} rfl_can_rx_msg_s;

extern const uint8_t *rflCan1AddRxMessageBox(uint32_t can_id);

extern const uint8_t *rflCan2AddRxMessageBox(uint32_t can_id);

extern void rflCanInit(void);

/* 使用RM官方电机 */
#if (RFL_DEV_MOTOR_RM_MOTOR == 1)

static CAN_TxHeaderTypeDef can_1_id0x200_tx_message;
static uint8_t can_1_id0x200_send_data[8];

extern void Can1RmMotorId0x200Control(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void Can1RmMotorId0x1ffControl(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

#endif /* RFL_DEV_MOTOR_RM_MOTOR == 1 */

#endif /* _DRV_CAN_H__ */
