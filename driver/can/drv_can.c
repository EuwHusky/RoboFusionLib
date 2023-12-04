#include "stdlib.h"
#include "string.h"

#include "drv_can.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

static rfl_can_rx_msg_s rfl_can_1_rx_msg_boxes[8] = {0};
static rfl_can_rx_msg_s rfl_can_2_rx_msg_boxes[8] = {0};

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
        switch (rx_header.StdId)
        {
        case 0x201:
            memcpy(rfl_can_1_rx_msg_boxes[0].rx_data, rx_data, 8 * sizeof(uint8_t));
            break;
        case 0x202:
            memcpy(rfl_can_1_rx_msg_boxes[1].rx_data, rx_data, 8 * sizeof(uint8_t));
            break;
        case 0x203:
            memcpy(rfl_can_1_rx_msg_boxes[2].rx_data, rx_data, 8 * sizeof(uint8_t));
            break;
        case 0x204:
            memcpy(rfl_can_1_rx_msg_boxes[3].rx_data, rx_data, 8 * sizeof(uint8_t));
            break;
        case 0x205:
            memcpy(rfl_can_1_rx_msg_boxes[4].rx_data, rx_data, 8 * sizeof(uint8_t));
            break;
        case 0x206:
            memcpy(rfl_can_1_rx_msg_boxes[5].rx_data, rx_data, 8 * sizeof(uint8_t));
            break;
        case 0x207:
            memcpy(rfl_can_1_rx_msg_boxes[6].rx_data, rx_data, 8 * sizeof(uint8_t));
            break;
        case 0x208:
            memcpy(rfl_can_1_rx_msg_boxes[7].rx_data, rx_data, 8 * sizeof(uint8_t));
            break;

        default:
            break;
        }
    }
    else if (&hcan2 == hcan)
    {
        switch (rx_header.StdId)
        {
        case 0x201:
            memcpy(rfl_can_2_rx_msg_boxes[0].rx_data, rx_data, 8 * sizeof(uint8_t));
            break;
        case 0x202:
            memcpy(rfl_can_2_rx_msg_boxes[1].rx_data, rx_data, 8 * sizeof(uint8_t));
            break;
        case 0x203:
            memcpy(rfl_can_2_rx_msg_boxes[2].rx_data, rx_data, 8 * sizeof(uint8_t));
            break;
        case 0x204:
            memcpy(rfl_can_2_rx_msg_boxes[3].rx_data, rx_data, 8 * sizeof(uint8_t));
            break;
        case 0x205:
            memcpy(rfl_can_2_rx_msg_boxes[4].rx_data, rx_data, 8 * sizeof(uint8_t));
            break;
        case 0x206:
            memcpy(rfl_can_2_rx_msg_boxes[5].rx_data, rx_data, 8 * sizeof(uint8_t));
            break;
        case 0x207:
            memcpy(rfl_can_2_rx_msg_boxes[6].rx_data, rx_data, 8 * sizeof(uint8_t));
            break;
        case 0x208:
            memcpy(rfl_can_2_rx_msg_boxes[7].rx_data, rx_data, 8 * sizeof(uint8_t));
            break;

        default:
            break;
        }
    }
}

const uint8_t *rflCan1AddRxMessageBox(uint32_t can_id)
{
    switch (can_id)
    {
    case 0x201:
        return rfl_can_1_rx_msg_boxes[0].rx_data;
    case 0x202:
        return rfl_can_1_rx_msg_boxes[1].rx_data;
    case 0x203:
        return rfl_can_1_rx_msg_boxes[2].rx_data;
    case 0x204:
        return rfl_can_1_rx_msg_boxes[3].rx_data;
    case 0x205:
        return rfl_can_1_rx_msg_boxes[4].rx_data;
    case 0x206:
        return rfl_can_1_rx_msg_boxes[5].rx_data;
    case 0x207:
        return rfl_can_1_rx_msg_boxes[6].rx_data;
    case 0x208:
        return rfl_can_1_rx_msg_boxes[7].rx_data;

    default:
        break;
    }

    return NULL;
}

const uint8_t *rflCan2AddRxMessageBox(uint32_t can_id)
{
    switch (can_id)
    {
    case 0x201:
        return rfl_can_2_rx_msg_boxes[0].rx_data;
    case 0x202:
        return rfl_can_2_rx_msg_boxes[1].rx_data;
    case 0x203:
        return rfl_can_2_rx_msg_boxes[2].rx_data;
    case 0x204:
        return rfl_can_2_rx_msg_boxes[3].rx_data;
    case 0x205:
        return rfl_can_2_rx_msg_boxes[4].rx_data;
    case 0x206:
        return rfl_can_2_rx_msg_boxes[5].rx_data;
    case 0x207:
        return rfl_can_2_rx_msg_boxes[6].rx_data;
    case 0x208:
        return rfl_can_2_rx_msg_boxes[7].rx_data;

    default:
        break;
    }

    return NULL;

    // static uint8_t msg_boxes_num = 0;

    // if (msg_boxes_num == 6) // 一路CAN最多挂载6个设备
    //     return NULL;

    // msg_boxes_num++;

    // rfl_can_2_rx_msg_boxes[msg_boxes_num - 1].can_id = can_id;

    // return rfl_can_2_rx_msg_boxes[msg_boxes_num - 1].rx_data;
}

void can_set_filter(void)
{
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
}

void can_start(void)
{
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void rflCanInit(void)
{
    can_set_filter();
    can_start();

    // rfl_can_1_rx_msg_boxes = (rfl_can_rx_msg_s *)malloc(6 * sizeof(rfl_can_rx_msg_s));
    // memset(rfl_can_1_rx_msg_boxes, 0, 6 * sizeof(rfl_can_1_rx_msg_boxes));

    // rfl_can_2_rx_msg_boxes = (rfl_can_rx_msg_s *)malloc(6 * sizeof(rfl_can_rx_msg_s));
    // memset(rfl_can_2_rx_msg_boxes, 0, 6 * sizeof(rfl_can_2_rx_msg_boxes));
}

/* 使用RM官方电机 */
#if (RFL_DEV_MOTOR_RM_MOTOR == 1)

static CAN_TxHeaderTypeDef can_1_id0x200_tx_message;
static uint8_t can_1_id0x200_send_data[8];
static CAN_TxHeaderTypeDef can_1_id0x1ff_tx_message;
static uint8_t can_1_id0x1ff_send_data[8];

void Can1RmMotorId0x200Control(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    can_1_id0x200_tx_message.StdId = 0x200;
    can_1_id0x200_tx_message.IDE = CAN_ID_STD;
    can_1_id0x200_tx_message.RTR = CAN_RTR_DATA;
    can_1_id0x200_tx_message.DLC = 0x08;
    can_1_id0x200_send_data[0] = motor1 >> 8;
    can_1_id0x200_send_data[1] = motor1;
    can_1_id0x200_send_data[2] = motor2 >> 8;
    can_1_id0x200_send_data[3] = motor2;
    can_1_id0x200_send_data[4] = motor3 >> 8;
    can_1_id0x200_send_data[5] = motor3;
    can_1_id0x200_send_data[6] = motor4 >> 8;
    can_1_id0x200_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan1, &can_1_id0x200_tx_message, can_1_id0x200_send_data, &send_mail_box);
}

void Can1RmMotorId0x1ffControl(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    can_1_id0x1ff_tx_message.StdId = 0x1ff;
    can_1_id0x1ff_tx_message.IDE = CAN_ID_STD;
    can_1_id0x1ff_tx_message.RTR = CAN_RTR_DATA;
    can_1_id0x1ff_tx_message.DLC = 0x08;
    can_1_id0x1ff_send_data[0] = motor1 >> 8;
    can_1_id0x1ff_send_data[1] = motor1;
    can_1_id0x1ff_send_data[2] = motor2 >> 8;
    can_1_id0x1ff_send_data[3] = motor2;
    can_1_id0x1ff_send_data[4] = motor3 >> 8;
    can_1_id0x1ff_send_data[5] = motor3;
    can_1_id0x1ff_send_data[6] = motor4 >> 8;
    can_1_id0x1ff_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan1, &can_1_id0x1ff_tx_message, can_1_id0x1ff_send_data, &send_mail_box);
}

#endif /* RFL_DEV_MOTOR_RM_MOTOR == 1 */
