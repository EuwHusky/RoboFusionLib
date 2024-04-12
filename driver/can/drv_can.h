#ifndef _DRV_CAN_H__
#define _DRV_CAN_H__

#include "stdint.h"

#include "rfl_config.h"

#define MAX_NUM_OF_RX_CAN_ID (12)

typedef struct RflCan
{
    uint8_t usage_size; /* 已使用的邮箱大小 */

    uint8_t storage[MAX_NUM_OF_RX_CAN_ID][8];                  /* 邮箱储存 */
    uint32_t id_table[MAX_NUM_OF_RX_CAN_ID];                   /* canid表 */
    void (*rx_callback_func_list[MAX_NUM_OF_RX_CAN_ID])(void); /* 接收触发回调函数 */
} rfl_can_rx_msg_box_s;

extern void rflCanInit(void);
extern void rflCanRxMessageBoxAddId(uint8_t can_ordinal, uint32_t can_id);
extern void rflCanRxMessageBoxAddRxCallbackFunc(uint8_t can_ordinal, uint32_t can_id, void (*rx_callback_func)(void));
extern void rflCanSendData(uint8_t can_ordinal, uint32_t can_id, uint8_t tx_data[8]);
extern uint8_t *rflCanGetRxMessageBoxData(uint8_t can_ordinal, uint32_t can_id);

/* 使用RM官方电机 */
#if (RFL_DEV_MOTOR_RM_MOTOR == 1)

extern void rflRmMotorControl(uint8_t can_ordinal, uint32_t can_id, int16_t motor1, int16_t motor2, int16_t motor3,
                              int16_t motor4);

#endif /* RFL_DEV_MOTOR_RM_MOTOR == 1 */

#endif /* _DRV_CAN_H__ */
