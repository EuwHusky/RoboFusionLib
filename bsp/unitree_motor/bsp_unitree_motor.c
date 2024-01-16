#include "bsp_unitree_motor.h"

#if (RFL_DEV_MOTOR_UNITREE_MOTOR == 1)

#include "math.h"
#include "stdio.h"
#include "string.h"

#include "main.h"

#include "cmsis_os.h"

#include "drv_usart.h"

#include "algo_angle.h"
#include "algo_crc_16_ccitt.h"

static void modify_data(unitree_motor_s *unitree_motor);
static uint8_t extract_data(unitree_motor_s *unitree_motor);

#define SATURATE(_IN, _MIN, _MAX)                                                                                      \
    {                                                                                                                  \
        if (_IN < _MIN)                                                                                                \
            _IN = _MIN;                                                                                                \
        else if (_IN > _MAX)                                                                                           \
            _IN = _MAX;                                                                                                \
    }

static void unitree_motor_control_delay(uint16_t ms)
{
    osDelay(ms);
}

void unitree_motor_init(unitree_motor_s *unitree_motor)
{
    // 发送一次停止命令以获取反馈数据
    unitree_motor->set_mode = 0;
    unitree_motor->set_torque = 0.0f;
    unitree_motor->set_speed = 0.0f;
    unitree_motor->set_angle = 0.0f;
    unitree_motor->k_a = 0.0f;
    unitree_motor->k_s = 0.0f;
    unitree_motor_control(unitree_motor);

    unitree_motor_control_delay(5);

    // 记录上电时的电机角度
    extract_data(unitree_motor);
    unitree_motor->angle_offset = unitree_motor->angle;
}

void unitree_motor_update_status(unitree_motor_s *unitree_motor)
{
    extract_data(unitree_motor);
}

void unitree_motor_control(unitree_motor_s *unitree_motor)
{
    modify_data(unitree_motor);

    usart1_tx_dma_enable((uint8_t *)(&(unitree_motor->command)), sizeof(unitree_motor_command_s));
}

void unitree_motor_reset_angle(unitree_motor_s *unitree_motor)
{
    unitree_motor->angle_offset = unitree_motor->angle;
}

void modify_data(unitree_motor_s *unitree_motor)
{
    unitree_motor->command.head[0] = 0xFE;
    unitree_motor->command.head[1] = 0xEE;

    // SATURATE(unitree_motor->targer_id, 0, 14);
    // SATURATE(unitree_motor->set_mode, 0, 2);
    SATURATE(unitree_motor->set_torque, -127.99f, 127.99f);
    SATURATE(unitree_motor->set_speed, -804.00f, 804.00f);
    SATURATE(unitree_motor->set_angle, -411774.0f, 411774.0f);
    SATURATE(unitree_motor->k_a, 0.0f, 25.599f);
    SATURATE(unitree_motor->k_s, 0.0f, 25.599f);

    unitree_motor->command.mode.id = unitree_motor->targer_id;
    unitree_motor->command.mode.status = unitree_motor->set_mode;

    unitree_motor->command.comd.tor_des = unitree_motor->set_torque * 256;
    unitree_motor->command.comd.spd_des = unitree_motor->set_speed / RAD_2_PI * 256;
    unitree_motor->command.comd.pos_des = unitree_motor->set_angle / RAD_2_PI * 32768;
    unitree_motor->command.comd.k_pos = unitree_motor->k_a / 25.6f * 32768;
    unitree_motor->command.comd.k_spd = unitree_motor->k_s / 25.6f * 32768;

    unitree_motor->command.CRC16 = crc_ccitt(0, (uint8_t *)&unitree_motor->command, 15);
}

uint8_t extract_data(unitree_motor_s *unitree_motor)
{
    if (unitree_motor->feedback->CRC16 != crc_ccitt(0, (uint8_t *)unitree_motor->feedback, 14))
    {
        // printf("[WARNING] Receive data CRC error");
        unitree_motor->is_data_correct = 0;

        return unitree_motor->is_data_correct;
    }
    else
    {
        unitree_motor->id = unitree_motor->feedback->mode.id;
        unitree_motor->mode = unitree_motor->feedback->mode.status;

        unitree_motor->torque = ((float)unitree_motor->feedback->fbk.torque) / 256;
        unitree_motor->speed = ((float)unitree_motor->feedback->fbk.speed / 256) * RAD_2_PI;
        unitree_motor->angle = RAD_2_PI * ((float)unitree_motor->feedback->fbk.pos) / 32768;
        unitree_motor->temp = unitree_motor->feedback->fbk.temp;
        unitree_motor->error_code = unitree_motor->feedback->fbk.MError;
        unitree_motor->foot_force = unitree_motor->feedback->fbk.force;

        unitree_motor->is_data_correct = 1;

        return unitree_motor->is_data_correct;
    }
}

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

#define UNITREE_MOTOR_FDB_FRAME_LENGTH (16U)
#define UNITREE_MOTOR_FDB_UART_RX_BUFFER_NUM (32U)
uint8_t usart_buf[2][UNITREE_MOTOR_FDB_UART_RX_BUFFER_NUM];

static uint16_t motor_id = 15;
unitree_motor_feedback_s unitree_motor_rx_data[15] = {0};

void unitree_uart_init(void)
{
    usart1_init(usart_buf[0], usart_buf[1], UNITREE_MOTOR_FDB_UART_RX_BUFFER_NUM);
}

const unitree_motor_feedback_s *unitree_motor_get_feedback_pointer(uint16_t unitree_motor_id)
{
    return &unitree_motor_rx_data[unitree_motor_id];
}

void USART1_IRQHandler(void)
{
    if (huart1.Instance->SR & UART_FLAG_RXNE) // 接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
    else if (USART1->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart1);

        if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET) /* Current memory buffer used is Memory 0 */
        {
            // disable DMA
            // 失效DMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            // get receive data length, length = set_data_length - remain_length
            // 获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = UNITREE_MOTOR_FDB_UART_RX_BUFFER_NUM - hdma_usart1_rx.Instance->NDTR;

            // reset set_data_lenght
            // 重新设定数据长度
            hdma_usart1_rx.Instance->NDTR = UNITREE_MOTOR_FDB_UART_RX_BUFFER_NUM;

            // set memory buffer 1
            // 设定缓冲区1
            hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;

            // enable DMA
            // 使能DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if (this_time_rx_len == UNITREE_MOTOR_FDB_FRAME_LENGTH)
            {
                if (usart_buf[0][0] == 0XFD && usart_buf[0][1] == 0XEE)
                {
                    motor_id = usart_buf[0][2] & 0XF;

                    if (motor_id < 15)
                    {
                        memcpy(&unitree_motor_rx_data[motor_id], usart_buf[0], sizeof(unitree_motor_feedback_s));
                    }
                }
            }
        }
        else /* Current memory buffer used is Memory 1 */
        {
            // disable DMA
            // 失效DMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            // get receive data length, length = set_data_length - remain_length
            // 获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = UNITREE_MOTOR_FDB_UART_RX_BUFFER_NUM - hdma_usart1_rx.Instance->NDTR;

            // reset set_data_lenght
            // 重新设定数据长度
            hdma_usart1_rx.Instance->NDTR = UNITREE_MOTOR_FDB_UART_RX_BUFFER_NUM;

            // set memory buffer 0
            // 设定缓冲区0
            DMA2_Stream5->CR &= ~(DMA_SxCR_CT);

            // enable DMA
            // 使能DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if (this_time_rx_len == UNITREE_MOTOR_FDB_FRAME_LENGTH)
            {
                if (usart_buf[1][0] == 0XFD && usart_buf[1][1] == 0XEE)
                {
                    motor_id = usart_buf[1][2] & 0XF;

                    if (motor_id < 15)
                    {
                        memcpy(&unitree_motor_rx_data[motor_id], usart_buf[1], sizeof(unitree_motor_feedback_s));
                    }
                }
            }
        }
    }
}

#endif /* RFL_DEV_MOTOR_UNITREE_MOTOR == 1 */
