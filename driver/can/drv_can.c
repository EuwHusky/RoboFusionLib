#include "stdlib.h"
#include "string.h"

#include "drv_can.h"

#if RFL_CONFIG_CORE == RFL_CORE_RM_C_BORAD
#define RFL_CAN_NUM RFL_CORE_RM_C_BORAD_CAN_NUM
#elif RFL_CONFIG_CORE == RFL_CORE_WPIE_HPM6750
#define RFL_CAN_NUM RFL_CORE_WPIE_HPM6750_CAN_NUM
#endif

static rfl_can_rx_msg_box_s *rfl_can_rx_message_boxes = NULL;

void rflCanRxMessageBoxesInit(void)
{
    rfl_can_rx_message_boxes = (rfl_can_rx_msg_box_s *)malloc(RFL_CAN_NUM * sizeof(rfl_can_rx_msg_box_s));

    for (uint8_t i = 0; i < RFL_CAN_NUM; i++)
    {
        rfl_can_rx_message_boxes[i].box_size = 0;
        rfl_can_rx_message_boxes[i].rx_msg_box = (rfl_can_rx_msg_s *)malloc(sizeof(rfl_can_rx_msg_s));
        rfl_can_rx_message_boxes[i].id_table = (uint32_t *)malloc(sizeof(uint32_t));
    }
}

void rflCanRxMessageBoxAddId(uint8_t can_ordinal, uint32_t can_id)
{
    rfl_can_rx_message_boxes[can_ordinal - 1].box_size++;
    rfl_can_rx_message_boxes[can_ordinal - 1].rx_msg_box =
        (rfl_can_rx_msg_s *)realloc(rfl_can_rx_message_boxes[can_ordinal - 1].rx_msg_box,
                                    rfl_can_rx_message_boxes[can_ordinal - 1].box_size * sizeof(rfl_can_rx_msg_s));
    rfl_can_rx_message_boxes[can_ordinal - 1].id_table =
        (uint32_t *)realloc(rfl_can_rx_message_boxes[can_ordinal - 1].id_table,
                            rfl_can_rx_message_boxes[can_ordinal - 1].box_size * sizeof(uint32_t));
    rfl_can_rx_message_boxes[can_ordinal - 1].id_table[rfl_can_rx_message_boxes[can_ordinal - 1].box_size - 1] = can_id;
}

void rflCanSaveToRxMessageBox(uint8_t can_ordinal, uint32_t can_id, uint8_t rx_data[8])
{
    for (uint8_t i = 0; i < rfl_can_rx_message_boxes[can_ordinal - 1].box_size; i++)
    {
        if (rfl_can_rx_message_boxes[can_ordinal - 1].id_table[i] == can_id)
        {
            memcpy(rfl_can_rx_message_boxes[can_ordinal - 1].rx_msg_box[i].data, rx_data, 8 * sizeof(uint8_t));
            break;
        }
    }
}

uint8_t *rflCanGetRxMessageBoxData(uint8_t can_ordinal, uint32_t can_id)
{
    for (uint8_t i = 0; i < rfl_can_rx_message_boxes[can_ordinal - 1].box_size; i++)
    {
        if (rfl_can_rx_message_boxes[can_ordinal - 1].id_table[i] == can_id)
            return rfl_can_rx_message_boxes[can_ordinal - 1].rx_msg_box[i].data;
    }

    return NULL;
}

#if RFL_CONFIG_CORE == RFL_CORE_RM_C_BORAD

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/**
 * @brief          hal CAN fifo call back, receive motor data
 * @param[in]      hcan, the point to CAN handle
 * @retval         none
 */
/**
 * @brief          hal库CAN回调函数,接收电机数据
 * @param[in]      hcan:CAN句柄指针
 * @retval         none
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if (&hcan1 == hcan)
    {
        rflCanSaveToRxMessageBox(1, rx_header.StdId, rx_data);
    }
    else if (&hcan2 == hcan)
    {
        rflCanSaveToRxMessageBox(2, rx_header.StdId, rx_data);
    }
}

void can_start(void)
{
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

#endif /* RFL_CONFIG_CORE == RFL_CORE_RM_C_BORAD */

void rflCanSendData(uint8_t can_ordinal, uint32_t can_id, uint8_t tx_data[8])
{
#if RFL_CONFIG_CORE == RFL_CORE_RM_C_BORAD
    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t send_mail_box;
    tx_header.StdId = can_id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 0x08;

    if (can_ordinal == 1)
        HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &send_mail_box);
    else if (can_ordinal == 2)
        HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, &send_mail_box);
#endif /* RFL_CONFIG_CORE == RFL_CORE_RM_C_BORAD */
}

void can_set_filter(void)
{
#if RFL_CONFIG_CORE == RFL_CORE_RM_C_BORAD
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
#endif /* RFL_CONFIG_CORE == RFL_CORE_RM_C_BORAD */
}

void rflCanInit(void)
{
    can_set_filter();

#if RFL_CONFIG_CORE == RFL_CORE_RM_C_BORAD
    can_start();
#endif /* RFL_CONFIG_CORE == RFL_CORE_RM_C_BORAD */

    rflCanRxMessageBoxesInit();

    rflCanRxMessageBoxAddId(1, 0x201);
    rflCanRxMessageBoxAddId(1, 0x202);
    rflCanRxMessageBoxAddId(1, 0x203);
    rflCanRxMessageBoxAddId(1, 0x204);
    rflCanRxMessageBoxAddId(1, 0x205);
    rflCanRxMessageBoxAddId(1, 0x206);
    rflCanRxMessageBoxAddId(1, 0x207);
    rflCanRxMessageBoxAddId(1, 0x208);

    rflCanRxMessageBoxAddId(2, 0x201);
    rflCanRxMessageBoxAddId(2, 0x202);
    rflCanRxMessageBoxAddId(2, 0x203);
    rflCanRxMessageBoxAddId(2, 0x204);
    rflCanRxMessageBoxAddId(2, 0x205);
    rflCanRxMessageBoxAddId(2, 0x206);
    rflCanRxMessageBoxAddId(2, 0x207);
    rflCanRxMessageBoxAddId(2, 0x208);
}

/* 使用RM官方电机 */
#if (RFL_DEV_MOTOR_RM_MOTOR == 1)

static uint8_t can_send_data[RFL_CAN_NUM][8];
void rflRmMotorControl(uint8_t can_ordinal, uint32_t can_id, int16_t motor1, int16_t motor2, int16_t motor3,
                       int16_t motor4)
{
    can_send_data[can_ordinal][0] = motor1 >> 8;
    can_send_data[can_ordinal][1] = motor1;
    can_send_data[can_ordinal][2] = motor2 >> 8;
    can_send_data[can_ordinal][3] = motor2;
    can_send_data[can_ordinal][4] = motor3 >> 8;
    can_send_data[can_ordinal][5] = motor3;
    can_send_data[can_ordinal][6] = motor4 >> 8;
    can_send_data[can_ordinal][7] = motor4;

    rflCanSendData(can_ordinal, can_id, can_send_data[can_ordinal]);
}

#endif /* RFL_DEV_MOTOR_RM_MOTOR == 1 */
