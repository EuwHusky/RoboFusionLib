#include "bsp_unitree_motor.h"

#if (RFL_DEV_MOTOR_UNITREE_MOTOR == 1)

#include "cmsis_os.h"

#include "drv_usart.h"

#include "algo_angle.h"
#include "algo_crc_16_ccitt.h"

#define CRC_SIZE 2
#define CTRL_DAT_SIZE (sizeof(unitree_motor_control_frame_s) - CRC_SIZE)
#define DATA_DAT_SIZE (sizeof(unitree_motor_feedback_frame_s) - CRC_SIZE)

#define SATURATE(_IN, _MIN, _MAX)                                                                                      \
    {                                                                                                                  \
        if (_IN < _MIN)                                                                                                \
            _IN = _MIN;                                                                                                \
        else if (_IN > _MAX)                                                                                           \
            _IN = _MAX;                                                                                                \
    }

int modify_data(unitree_motor_s *unitree_motor)
{
    unitree_motor->command.head[0] = 0xFE;
    unitree_motor->command.head[1] = 0xEE;

    // SATURATE(unitree_motor->targer_id, 0, 15);
    // SATURATE(unitree_motor->set_mode, 0, 7);
    SATURATE(unitree_motor->set_torque, -127.99f, 127.99f);
    SATURATE(unitree_motor->set_speed, -804.00f, 804.00f);
    SATURATE(unitree_motor->set_angle, -411774.0f, 411774.0f);
    SATURATE(unitree_motor->k_s, 0.0f, 25.599f);
    SATURATE(unitree_motor->k_a, 0.0f, 25.599f);

    unitree_motor->command.mode.id = unitree_motor->targer_id;
    unitree_motor->command.mode.status = unitree_motor->set_mode;

    unitree_motor->command.comd.tor_des = unitree_motor->set_torque;
    unitree_motor->command.comd.spd_des = unitree_motor->set_speed;
    unitree_motor->command.comd.pos_des = unitree_motor->set_angle;
    unitree_motor->command.comd.k_spd = unitree_motor->k_s;
    unitree_motor->command.comd.k_pos = unitree_motor->k_a;

    unitree_motor->command.CRC16 = crc_ccitt(0, (uint8_t *)&unitree_motor->command, 15);

    return 0;
}

int extract_data(unitree_motor_s *unitree_motor)
{
    if (unitree_motor->feedback.CRC16 != crc_ccitt(0, (uint8_t *)&unitree_motor->feedback, 14))
    {
        // printf("[WARNING] Receive data CRC error");
        unitree_motor->is_data_correct = 0;

        return unitree_motor->is_data_correct;
    }
    else
    {
        unitree_motor->id = unitree_motor->feedback.mode.id;
        unitree_motor->mode = unitree_motor->feedback.mode.status;

        unitree_motor->torque = ((float)unitree_motor->feedback.fbk.torque) / 256;
        unitree_motor->speed = ((float)unitree_motor->feedback.fbk.speed / 256) * RAD_2_PI;
        unitree_motor->angle = RAD_2_PI * ((float)unitree_motor->feedback.fbk.pos) / 32768;
        unitree_motor->temp = unitree_motor->feedback.fbk.temp;
        unitree_motor->error_code = unitree_motor->feedback.fbk.MError;
        unitree_motor->foot_force = unitree_motor->feedback.fbk.force;

        unitree_motor->is_data_correct = 1;

        return unitree_motor->is_data_correct;
    }
}

extern UART_HandleTypeDef huart6;
void unitree_motor_control(unitree_motor_s *unitree_motor)
{
    modify_data(unitree_motor);

    SET_485_DE_UP();
    SET_485_RE_UP();

    HAL_UART_Transmit(&huart6, (uint8_t *)unitree_motor, sizeof(unitree_motor->command), 20);

    SET_485_RE_DOWN();
    SET_485_DE_DOWN();

    osDelay(1);
}

// int modify_data(MOTOR_send *motor_s)
// {
//     motor_s->motor_send_data.head[0] = 0xFE;
//     motor_s->motor_send_data.head[1] = 0xEE;

//     // SATURATE(motor_s->id, 0, 15);
//     // SATURATE(motor_s->mode, 0, 7);
//     SATURATE(motor_s->K_P, 0.0f, 25.599f);
//     SATURATE(motor_s->K_W, 0.0f, 25.599f);
//     SATURATE(motor_s->T, -127.99f, 127.99f);
//     SATURATE(motor_s->W, -804.00f, 804.00f);
//     SATURATE(motor_s->Pos, -411774.0f, 411774.0f);

//     motor_s->motor_send_data.mode.id = motor_s->id;
//     motor_s->motor_send_data.mode.status = motor_s->mode;

//     motor_s->motor_send_data.comd.k_pos = motor_s->K_P / 25.6f * 32768;
//     motor_s->motor_send_data.comd.k_spd = motor_s->K_W / 25.6f * 32768;
//     motor_s->motor_send_data.comd.pos_des = motor_s->Pos / 6.2832f * 32768;
//     motor_s->motor_send_data.comd.spd_des = motor_s->W / 6.2832f * 256;
//     motor_s->motor_send_data.comd.tor_des = motor_s->T * 256;

//     motor_s->motor_send_data.CRC16 = crc_ccitt(0, (uint8_t *)&motor_s->motor_send_data, 15);

//     return 0;
// }

// int extract_data(MOTOR_recv *motor_r)
// {
//     if (motor_r->motor_recv_data.CRC16 != crc_ccitt(0, (uint8_t *)&motor_r->motor_recv_data, 14))
//     {
//         // printf("[WARNING] Receive data CRC error");
//         motor_r->is_data_correct = 0;

//         return motor_r->is_data_correct;
//     }
//     else
//     {
//         motor_r->id = motor_r->motor_recv_data.mode.id;
//         motor_r->mode = motor_r->motor_recv_data.mode.status;
//         motor_r->Temp = motor_r->motor_recv_data.fbk.temp;
//         motor_r->MError = motor_r->motor_recv_data.fbk.MError;
//         motor_r->W = ((float)motor_r->motor_recv_data.fbk.speed / 256) * 6.2832f;
//         motor_r->T = ((float)motor_r->motor_recv_data.fbk.torque) / 256;
//         motor_r->Pos = 6.2832f * ((float)motor_r->motor_recv_data.fbk.pos) / 32768;
//         motor_r->footForce = motor_r->motor_recv_data.fbk.force;
//         motor_r->is_data_correct = 1;

//         return motor_r->is_data_correct;
//     }
// }

// extern UART_HandleTypeDef huart6;
// void SERVO_Send(MOTOR_send *pData)
// {
//     modify_data(pData);

//     SET_485_DE_UP();
//     SET_485_RE_UP();

//     HAL_UART_Transmit(&huart6, (uint8_t *)pData, sizeof(pData->motor_send_data), 20);

//     SET_485_RE_DOWN();
//     SET_485_DE_DOWN();

//     osDelay(1);
// }

#endif /* RFL_DEV_MOTOR_UNITREE_MOTOR == 1 */
