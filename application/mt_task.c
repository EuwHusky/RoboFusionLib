#include "stdio.h"
#include "string.h"

#include "mt_task.h"

#include "main.h"

#include "cmsis_os.h"

#include "drv_can.h"
#include "drv_usart.h"

#include "bsp_led.h"

#include "dev_motor.h"

#include "algo_crc_16_ccitt.h"

extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
#define UNITREE_MOTOR_FDB_FRAME_LENGTH (16U)
#define UNITREE_MOTOR_FDB_UART_RX_BUFFER_NUM (32U)

uint8_t usart6_buf[2][UNITREE_MOTOR_FDB_UART_RX_BUFFER_NUM];

// MOTOR_recv rx_unitree_motor_data = {0};

unitree_motor_s test_unitree_motor[2] = {0};

rfl_motor_config_s test_motor_config = {0};
rfl_motor_s test_gm6020 = {0};

void mt_task(void const *pvParameters)
{
    osDelay(1000);

    rflMotorGetDefaultConfig(&test_motor_config, RFL_MOTOR_RM_GM6020);
    test_motor_config.mode = RFL_MOTOR_CONTROL_MODE_SPEED;
    test_motor_config.max_accle = 8;
    test_motor_config.can_handle_id = 1;
    test_motor_config.can_id = 0x205;
    rflMotorInit(&test_gm6020, &test_motor_config);
    rflMotorSetSpeed(&test_gm6020, 6);

    while (1)
    {
        // test_unitree_motor[0].id = 0;
        // test_unitree_motor[0].mode = 0;
        // test_unitree_motor[0].T = 0.0f;
        // test_unitree_motor[0].W = 6.28f * 6.33f;
        // test_unitree_motor[0].Pos = 0.0f;
        // test_unitree_motor[0].K_P = 0.0f;
        // test_unitree_motor[0].K_W = 0.05f;
        // SERVO_Send(&test_unitree_motor[0]);
        // osDelay(1);

        // test_unitree_motor[1].id = 1;
        // test_unitree_motor[1].mode = 0;
        // test_unitree_motor[1].T = 0.0f;
        // test_unitree_motor[1].W = -6.28f * 6.33f;
        // test_unitree_motor[1].Pos = 0.0f;
        // test_unitree_motor[1].K_P = 0.0f;
        // test_unitree_motor[1].K_W = 0.05f;
        // SERVO_Send(&test_unitree_motor[1]);
        // osDelay(1);

        rflMotorUpdateStatus(&test_gm6020);
        rflMotorUpdataControl(&test_gm6020);
        Can1RmMotorId0x1ffControl((int16_t)rflMotorGetOutput(&test_gm6020), 0, 0, 0);

        ArgbLedControl(0xff00ff00);
        osDelay(2);
    }
}

void test_task(void const *argument)
{
    osDelay(100);

    while (1)
    {
        osDelay(200);
    }
    /* USER CODE END test_task */
}

void USART6_IRQHandler(void)
{
    if (huart6.Instance->SR & UART_FLAG_RXNE) // 接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);
    }
    else if (USART6->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart6);

        if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET) /* Current memory buffer used is Memory 0 */
        {
            // disable DMA
            // 失效DMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);

            // get receive data length, length = set_data_length - remain_length
            // 获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = UNITREE_MOTOR_FDB_UART_RX_BUFFER_NUM - hdma_usart6_rx.Instance->NDTR;

            // reset set_data_lenght
            // 重新设定数据长度
            hdma_usart6_rx.Instance->NDTR = UNITREE_MOTOR_FDB_UART_RX_BUFFER_NUM;

            // set memory buffer 1
            // 设定缓冲区1
            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;

            // enable DMA
            // 使能DMA
            __HAL_DMA_ENABLE(&hdma_usart6_rx);

            if (this_time_rx_len == UNITREE_MOTOR_FDB_FRAME_LENGTH)
            {
                // uint16_t motor_id = ((MOTOR_recv *)&usart6_buf[0])->motor_recv_data.mode.id;
                // if (motor_id == 0)
                // {
                //     memcpy((uint8_t *)&test_unitree_motor_data[0], usart6_buf[0],
                //            sizeof(test_unitree_motor_data[0].motor_recv_data));
                //     extract_data(&test_unitree_motor_data[0]);
                // }
                // else if (motor_id == 1)
                // {
                //     memcpy((uint8_t *)&test_unitree_motor_data[1], usart6_buf[0],
                //            sizeof(test_unitree_motor_data[1].motor_recv_data));
                //     extract_data(&test_unitree_motor_data[1]);
                // }
                // // 记录数据接收时间
                // detect_hook(DBUS_TOE);
                // sbus_to_usart1(sbus_rx_buf[0]);
            }
        }
        else /* Current memory buffer used is Memory 1 */
        {
            // disable DMA
            // 失效DMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);

            // get receive data length, length = set_data_length - remain_length
            // 获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = UNITREE_MOTOR_FDB_UART_RX_BUFFER_NUM - hdma_usart6_rx.Instance->NDTR;

            // reset set_data_lenght
            // 重新设定数据长度
            hdma_usart6_rx.Instance->NDTR = UNITREE_MOTOR_FDB_UART_RX_BUFFER_NUM;

            // set memory buffer 0
            // 设定缓冲区0
            // hdma_usart6_rx.Instance->CR &= ~(DMA_SxCR_CT);
            DMA2_Stream1->CR &= ~(DMA_SxCR_CT);

            // enable DMA
            // 使能DMA
            __HAL_DMA_ENABLE(&hdma_usart6_rx);

            if (this_time_rx_len == UNITREE_MOTOR_FDB_FRAME_LENGTH)
            {
                // uint16_t motor_id = ((MOTOR_recv *)&usart6_buf[1])->motor_recv_data.mode.id;
                // if (motor_id == 0)
                // {
                //     memcpy((uint8_t *)&test_unitree_motor_data[0], usart6_buf[1],
                //            sizeof(test_unitree_motor_data[0].motor_recv_data));
                //     extract_data(&test_unitree_motor_data[0]);
                // }
                // else if (motor_id == 1)
                // {
                //     memcpy((uint8_t *)&test_unitree_motor_data[1], usart6_buf[1],
                //            sizeof(test_unitree_motor_data[1].motor_recv_data));
                //     extract_data(&test_unitree_motor_data[1]);
                // }
                // // 记录数据接收时间
                // detect_hook(DBUS_TOE);
                // sbus_to_usart1(sbus_rx_buf[1]);
            }
        }
    }
}

void unitree_uart_init(void)
{
    usart6_init(usart6_buf[0], usart6_buf[1], UNITREE_MOTOR_FDB_UART_RX_BUFFER_NUM);
}
