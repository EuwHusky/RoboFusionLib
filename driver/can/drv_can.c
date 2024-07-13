#include "stdlib.h"
#include "string.h"

#include "drv_can.h"

#if RFL_CONFIG_CORE == RFL_CORE_WPIE_HPM6750
#include "board.h"
#include "hpm_can_drv.h"
#include "hpm_interrupt.h"

#define RFL_CAN_NUM RFL_CORE_WPIE_HPM6750_CAN_NUM

#elif RFL_CONFIG_CORE == RFL_CORE_RM_C_BORAD
#include "main.h"

#define RFL_CAN_NUM RFL_CORE_RM_C_BORAD_CAN_NUM

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

#endif

#if RFL_CONFIG_CORE == RFL_CORE_WPIE_HPM6750
static CAN_Type *get_wpie_hpm6750_can_id(uint8_t can_ordinal);
#endif

static rfl_can_rx_msg_box_s *rx_message_boxes = NULL;

void rfl_can_rx_message_boxes_init(void)
{
    rx_message_boxes = (rfl_can_rx_msg_box_s *)malloc(RFL_CAN_NUM * sizeof(rfl_can_rx_msg_box_s));
    memset(rx_message_boxes, 0, RFL_CAN_NUM * sizeof(rfl_can_rx_msg_box_s));

    for (uint8_t i = 0; i < RFL_CAN_NUM; i++)
        for (uint8_t j = 0; j < MAX_NUM_OF_RX_CAN_ID; j++)
            rx_message_boxes[i].rx_callback_func_list[j] = NULL;
}

void rflCanRxMessageBoxAddId(uint8_t can_ordinal, uint32_t can_id)
{
    if (can_ordinal > RFL_CAN_NUM)
        return;
    if (rx_message_boxes[can_ordinal - 1].usage_size >= MAX_NUM_OF_RX_CAN_ID)
        return;
    for (uint8_t i = 0; i < rx_message_boxes[can_ordinal - 1].usage_size; i++)
        if (rx_message_boxes[can_ordinal - 1].id_table[i] == can_id)
            return;

    rx_message_boxes[can_ordinal - 1].usage_size++;
    rx_message_boxes[can_ordinal - 1].id_table[rx_message_boxes[can_ordinal - 1].usage_size - 1] = can_id;
}

void rflCanRxMessageBoxAddRxCallbackFunc(uint8_t can_ordinal, uint32_t can_id, void (*rx_callback_func)(void))
{
    if (can_ordinal > RFL_CAN_NUM)
        return;

    for (uint8_t i = 0; i < rx_message_boxes[can_ordinal - 1].usage_size; i++)
    {
        if (rx_message_boxes[can_ordinal - 1].id_table[i] == can_id)
        {
            rx_message_boxes[can_ordinal - 1].rx_callback_func_list[i] = rx_callback_func;
            break;
        }
    }
}

void rflCanSaveToRxMessageBox(uint8_t can_ordinal, uint32_t can_id, uint8_t rx_data[8])
{
    if (can_ordinal > RFL_CAN_NUM)
        return;

    for (uint8_t i = 0; i < rx_message_boxes[can_ordinal - 1].usage_size; i++)
    {
        if (rx_message_boxes[can_ordinal - 1].id_table[i] == can_id)
        {
            memcpy(rx_message_boxes[can_ordinal - 1].storage[i], rx_data, 8 * sizeof(uint8_t));
            if (rx_message_boxes[can_ordinal - 1].rx_callback_func_list[i] != NULL)
                rx_message_boxes[can_ordinal - 1].rx_callback_func_list[i]();
            break;
        }
    }
}

uint8_t *rflCanGetRxMessageBoxData(uint8_t can_ordinal, uint32_t can_id)
{
    if (can_ordinal > RFL_CAN_NUM)
        return NULL;

    for (uint8_t i = 0; i < rx_message_boxes[can_ordinal - 1].usage_size; i++)
    {
        if (rx_message_boxes[can_ordinal - 1].id_table[i] == can_id)
        {
            return rx_message_boxes[can_ordinal - 1].storage[i];
        }
    }

    return NULL;
}

void rflCanSendData(uint8_t can_ordinal, uint32_t can_id, uint8_t tx_data[8])
{
    if (can_ordinal > RFL_CAN_NUM)
        return;

#if RFL_CONFIG_CORE == RFL_CORE_WPIE_HPM6750
    can_transmit_buf_t tx_buf;
    memset(&tx_buf, 0, sizeof(can_transmit_buf_t));
    tx_buf.dlc = can_payload_size_8;
    tx_buf.id = can_id;
    memcpy(tx_buf.data, tx_data, 8 * sizeof(uint8_t));
    for (uint16_t i = 0; i < 10000; i++) // 检查发送缓冲区是否已满，满则循环延时
        if (!can_is_secondary_transmit_buffer_full(get_wpie_hpm6750_can_id(can_ordinal)))
            break;
    if (can_ordinal == 1)
        can_send_message_nonblocking(BOARD_CAN1, &tx_buf);
    else if (can_ordinal == 2)
        can_send_message_nonblocking(BOARD_CAN2, &tx_buf);
    else if (can_ordinal == 3)
        can_send_message_nonblocking(BOARD_CAN3, &tx_buf);
    else if (can_ordinal == 4)
        can_send_message_nonblocking(BOARD_CAN4, &tx_buf);

#elif RFL_CONFIG_CORE == RFL_CORE_RM_C_BORAD
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

#endif
}

#if RFL_CONFIG_CORE == RFL_CORE_WPIE_HPM6750

static volatile uint8_t can_error_flags[4]; // 错误中断标志
static volatile bool can_has_error[4];      // 出现错误

/**
 * @brief CAN1中断服务函数
 */
void can1_callback(void)
{
    uint8_t can = can_get_tx_rx_flags(BOARD_CAN1); // 获取CAN标志位
    //  如果接收到消息
    if ((can & CAN_EVENT_RECEIVE) != 0)
    {
        can_receive_buf_t rx_buf; // 接收缓冲
        memset(&rx_buf, 0, sizeof(can_receive_buf_t));
        can_read_received_message(BOARD_CAN1, (can_receive_buf_t *)&rx_buf);

        rflCanSaveToRxMessageBox(1, rx_buf.id, rx_buf.data);
    }
    can_clear_tx_rx_flags(BOARD_CAN1, can); // 清除标志位
    // 如果出现错误,更新标志位并重启can
    if ((can & CAN_EVENT_ERROR) != 0)
    {
        can_has_error[0] = true;
        can_reset(BOARD_CAN1, false);
    }
    can_error_flags[0] = can_get_error_interrupt_flags(BOARD_CAN1);
    can_clear_error_interrupt_flags(BOARD_CAN1, can_error_flags[0]);
}

/**
 * @brief CAN2中断服务函数
 */
void can2_callback(void)
{
    uint8_t can = can_get_tx_rx_flags(BOARD_CAN2); // 获取CAN标志位
    //  如果接收到消息
    if ((can & CAN_EVENT_RECEIVE) != 0)
    {
        can_receive_buf_t rx_buf; // 接收缓冲
        memset(&rx_buf, 0, sizeof(can_receive_buf_t));
        can_read_received_message(BOARD_CAN2, (can_receive_buf_t *)&rx_buf);

        rflCanSaveToRxMessageBox(2, rx_buf.id, rx_buf.data);
    }
    can_clear_tx_rx_flags(BOARD_CAN2, can); // 清除标志位
    // 如果出现错误,更新标志位并重启can
    if ((can & CAN_EVENT_ERROR) != 0)
    {
        can_has_error[1] = true;
        can_reset(BOARD_CAN2, false);
    }
    can_error_flags[1] = can_get_error_interrupt_flags(BOARD_CAN2);
    can_clear_error_interrupt_flags(BOARD_CAN2, can_error_flags[1]);
}

/**
 * @brief CAN3中断服务函数
 */
void can3_callback(void)
{
    uint8_t can = can_get_tx_rx_flags(BOARD_CAN3); // 获取CAN标志位
    //  如果接收到消息
    if ((can & CAN_EVENT_RECEIVE) != 0)
    {
        can_receive_buf_t rx_buf; // 接收缓冲
        memset(&rx_buf, 0, sizeof(can_receive_buf_t));
        can_read_received_message(BOARD_CAN3, (can_receive_buf_t *)&rx_buf);

        rflCanSaveToRxMessageBox(3, rx_buf.id, rx_buf.data);
    }
    can_clear_tx_rx_flags(BOARD_CAN3, can); // 清除标志位
    // 如果出现错误,更新标志位并重启can
    if ((can & CAN_EVENT_ERROR) != 0)
    {
        can_has_error[2] = true;
        can_reset(BOARD_CAN3, false);
    }
    can_error_flags[2] = can_get_error_interrupt_flags(BOARD_CAN3);
    can_clear_error_interrupt_flags(BOARD_CAN3, can_error_flags[2]);
}

/**
 * @brief CAN4中断服务函数
 */
void can4_callback(void)
{
    uint8_t can = can_get_tx_rx_flags(BOARD_CAN4); // 获取CAN标志位
    //  如果接收到消息
    if ((can & CAN_EVENT_RECEIVE) != 0)
    {
        can_receive_buf_t rx_buf; // 接收缓冲
        memset(&rx_buf, 0, sizeof(can_receive_buf_t));
        can_read_received_message(BOARD_CAN4, (can_receive_buf_t *)&rx_buf);

        rflCanSaveToRxMessageBox(4, rx_buf.id, rx_buf.data);
    }
    can_clear_tx_rx_flags(BOARD_CAN4, can); // 清除标志位
    // 如果出现错误,更新标志位并重启can
    if ((can & CAN_EVENT_ERROR) != 0)
    {
        can_has_error[3] = true;
        can_reset(BOARD_CAN4, false);
    }
    can_error_flags[3] = can_get_error_interrupt_flags(BOARD_CAN4);
    can_clear_error_interrupt_flags(BOARD_CAN4, can_error_flags[3]);
}

SDK_DECLARE_EXT_ISR_M(BOARD_CAN1_IRQn, can1_callback);
SDK_DECLARE_EXT_ISR_M(BOARD_CAN2_IRQn, can2_callback);
SDK_DECLARE_EXT_ISR_M(BOARD_CAN3_IRQn, can3_callback);
SDK_DECLARE_EXT_ISR_M(BOARD_CAN4_IRQn, can4_callback);

#elif RFL_CONFIG_CORE == RFL_CORE_RM_C_BORAD

/**
 * @brief          hal库CAN回调函数,接收电机数据
 * @param[in]      hcan:CAN句柄指针
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

#endif

void rflCanInit(void)
{
#if RFL_CONFIG_CORE == RFL_CORE_WPIE_HPM6750
    board_init_can(BOARD_CAN1);                          // 初始化CAN1引脚
    board_init_can(BOARD_CAN2);                          // 初始化CAN1引脚
    board_init_can(BOARD_CAN3);                          // 初始化CAN3引脚
    board_init_can(BOARD_CAN4);                          // 初始化CAN4引脚
    can_config_t can_config;                             // CAN配置结构体
    can_get_default_config(&can_config);                 // 获取默认配置
    can_config.baudrate = BOARD_CAN_BAUDRATE;            // 默认波特率
    can_config.mode = can_mode_normal;                   // 普通模式
    can_config.irq_txrx_enable_mask = CAN_EVENT_RECEIVE; // 接收中断

    uint32_t can_clock_freq; // 时钟频率
    hpm_stat_t status;
    can_clock_freq = board_init_can_clock(BOARD_CAN1);
    if (status = can_init(BOARD_CAN1, &can_config, can_clock_freq), status != status_success)
    {
        printf("CAN 1 initialization failed, error code: %d\n", status);
    }
    can_clock_freq = board_init_can_clock(BOARD_CAN2);
    if (status = can_init(BOARD_CAN2, &can_config, can_clock_freq), status != status_success)
    {
        printf("CAN 2 initialization failed, error code: %d\n", status);
    }
    can_clock_freq = board_init_can_clock(BOARD_CAN3);
    if (status = can_init(BOARD_CAN3, &can_config, can_clock_freq), status != status_success)
    {
        printf("CAN 3 initialization failed, error code: %d\n", status);
    }
    can_clock_freq = board_init_can_clock(BOARD_CAN4);
    if (status = can_init(BOARD_CAN4, &can_config, can_clock_freq), status != status_success)
    {
        printf("CAN 4 initialization failed, error code: %d\n", status);
    }

    intc_m_enable_irq_with_priority(BOARD_CAN1_IRQn, 1);
    intc_m_enable_irq_with_priority(BOARD_CAN2_IRQn, 1);
    intc_m_enable_irq_with_priority(BOARD_CAN3_IRQn, 1);
    intc_m_enable_irq_with_priority(BOARD_CAN4_IRQn, 1);

#elif RFL_CONFIG_CORE == RFL_CORE_RM_C_BORAD
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

    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

#endif

    rfl_can_rx_message_boxes_init();
}

/* 使用RM官方电机 */
#if RFL_BSP_RM_MOTOR_ENABLED

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

#if RFL_CONFIG_CORE == RFL_CORE_WPIE_HPM6750
static CAN_Type *get_wpie_hpm6750_can_id(uint8_t can_ordinal)
{
    if (can_ordinal == 1)
        return HPM_CAN0;
    else if (can_ordinal == 2)
        return HPM_CAN1;
    else if (can_ordinal == 3)
        return HPM_CAN2;
    else if (can_ordinal == 4)
        return HPM_CAN3;
    else
        return NULL;
}
#endif

#endif /* RFL_BSP_RM_MOTOR_ENABLED */
