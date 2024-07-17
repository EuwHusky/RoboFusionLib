#ifndef _BSP_DT7_DR16_H__
#define _BSP_DT7_DR16_H__

#include "stdbool.h"
#include "stdint.h"

#include "rfl_config.h"

#if RFL_DEV_REMOTE_CONTROLL_DT7_DR16

typedef enum
{
    DT7_ROCKER_LEFT_HORIZONTAL = 0,
    DT7_ROCKER_LEFT_VERTICAL_SWITCH = 1,
    DT7_ROCKER_RIGHT_HORIZONTAL_SWITCH = 2,
    DT7_ROCKER_RIGHT_VERTICAL_SWITCH = 3
} dt7_rocker_channel_index_e;

typedef enum
{
    DT7_SWITCH_LEFT = 1,
    DT7_SWITCH_RIGHT = 0
} dt7_toggle_switch_index_e;

typedef enum
{
    DT7_SWITCH_NONE = 0,
    DT7_SWITCH_UP = 1,
    DT7_SWITCH_MID = 3,
    DT7_SWITCH_DOWN = 2
} dt7_toggle_switch_position_e;

typedef struct __attribute__((packed))
{
    struct __attribute__((packed))
    {
        int16_t ch[5]; // 遥控器遥感&拨盘
        char s[2];     // 遥控器拨杆
    } rc;
    struct __attribute__((packed))
    {
        int16_t x;       // 鼠标x
        int16_t y;       // 鼠标y
        int16_t z;       // 鼠标滚轮
        uint8_t press_l; // 鼠标左键
        uint8_t press_r; // 鼠标右键
    } mouse;
    struct __attribute__((packed))
    {
        uint16_t v; // 16个按键数据
    } key;
} rfl_dt7_dr16_data_s;

#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)

#define DT7_DR16_RC_CH_VALUE_MIN ((uint16_t)364)
#define DT7_DR16_RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define DT7_DR16_RC_CH_VALUE_MAX ((uint16_t)1684)

extern void rflDt7Dr16Decode(volatile const uint8_t *data_buf, rfl_dt7_dr16_data_s *dt7_dr16_data);
extern bool rflDt7Dr16CheckIsDataCorrect(rfl_dt7_dr16_data_s *dt7_dr16_data);

#endif /* RFL_DEV_REMOTE_CONTROLL_DT7_DR16 */

#endif /* _BSP_DT7_DR16_H__ */
