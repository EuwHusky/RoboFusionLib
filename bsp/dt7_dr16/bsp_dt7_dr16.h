#ifndef _BSP_DT7_DR16_H__
#define _BSP_DT7_DR16_H__

#include "stdint.h"

#include "rfl_config.h"

#if RFL_DEV_REMOTE_CONTROLL_DT7_DR16

#if RFL_CONFIG_CORE == RFL_CORE_WPIE_HPM6750

#include "board.h"

typedef struct
{
    struct
    {
        int16_t ch[5]; // 遥控器遥感&拨盘
        char s[2];     // 遥控器拨杆
    } __packed rc;
    struct
    {
        int16_t x;       // 鼠标x
        int16_t y;       // 鼠标y
        int16_t z;       // 鼠标滚轮
        uint8_t press_l; // 鼠标左键
        uint8_t press_r; // 鼠标右键
    } __packed mouse;
    struct
    {
        uint16_t v; // 16个按键数据
    } __packed key;
} __packed rfl_dt7_dr16_data_s;

#endif /* RFL_CONFIG_CORE == RFL_CORE_WPIE_HPM6750 */

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
