#include "bsp_dt7_dr16.h"

#include "algo_math.h"

#if RFL_DEV_REMOTE_CONTROLL_DT7_DR16

void rflDt7Dr16Decode(volatile const uint8_t *data_buf, rfl_dt7_dr16_data_s *dt7_dr16_data)
{
    if (data_buf == NULL || dt7_dr16_data == NULL)
    {
        return;
    }

    dt7_dr16_data->rc.ch[0] = (data_buf[0] | (data_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    dt7_dr16_data->rc.ch[1] = ((data_buf[1] >> 3) | (data_buf[2] << 5)) & 0x07ff; //!< Channel 1
    dt7_dr16_data->rc.ch[2] = ((data_buf[2] >> 6) | (data_buf[3] << 2) |          //!< Channel 2
                               (data_buf[4] << 10)) &
                              0x07ff;
    dt7_dr16_data->rc.ch[3] = ((data_buf[4] >> 1) | (data_buf[5] << 7)) & 0x07ff; //!< Channel 3
    dt7_dr16_data->rc.s[0] = ((data_buf[5] >> 4) & 0x0003);                       //!< Switch left
    dt7_dr16_data->rc.s[1] = ((data_buf[5] >> 4) & 0x000C) >> 2;                  //!< Switch right
    dt7_dr16_data->mouse.x = data_buf[6] | (data_buf[7] << 8);                    //!< Mouse X axis
    dt7_dr16_data->mouse.y = data_buf[8] | (data_buf[9] << 8);                    //!< Mouse Y axis
    dt7_dr16_data->mouse.z = data_buf[10] | (data_buf[11] << 8);                  //!< Mouse Z axis
    dt7_dr16_data->mouse.press_l = data_buf[12];                                  //!< Mouse Left Is Press ?
    dt7_dr16_data->mouse.press_r = data_buf[13];                                  //!< Mouse Right Is Press ?
    dt7_dr16_data->key.v = data_buf[14] | (data_buf[15] << 8);                    //!< KeyBoard value
    dt7_dr16_data->rc.ch[4] = (data_buf[16] | (data_buf[17] << 8)) & 0x07ff;      //

    dt7_dr16_data->rc.ch[0] -= DT7_DR16_RC_CH_VALUE_OFFSET;
    dt7_dr16_data->rc.ch[1] -= DT7_DR16_RC_CH_VALUE_OFFSET;
    dt7_dr16_data->rc.ch[2] -= DT7_DR16_RC_CH_VALUE_OFFSET;
    dt7_dr16_data->rc.ch[3] -= DT7_DR16_RC_CH_VALUE_OFFSET;
    dt7_dr16_data->rc.ch[4] -= DT7_DR16_RC_CH_VALUE_OFFSET;
}

// 判断遥控器数据是否出错
bool rflDt7Dr16CheckIsDataCorrect(rfl_dt7_dr16_data_s *dt7_dr16_data)
{
    // 使用了go to语句 方便出错统一处理遥控器变量数据归零
    if (rflAbsInt16(dt7_dr16_data->rc.ch[0]) > 660)
    {
        goto error;
    }
    if (rflAbsInt16(dt7_dr16_data->rc.ch[1]) > 660)
    {
        goto error;
    }
    if (rflAbsInt16(dt7_dr16_data->rc.ch[2]) > 660)
    {
        goto error;
    }
    if (rflAbsInt16(dt7_dr16_data->rc.ch[3]) > 660)
    {
        goto error;
    }
    if (rflAbsInt16(dt7_dr16_data->rc.ch[4]) > 660)
    {
        goto error;
    }
    if (dt7_dr16_data->rc.s[0] == 0)
    {
        goto error;
    }
    if (dt7_dr16_data->rc.s[1] == 0)
    {
        goto error;
    }
    return true;

error:
    dt7_dr16_data->rc.ch[0] = 0;
    dt7_dr16_data->rc.ch[1] = 0;
    dt7_dr16_data->rc.ch[2] = 0;
    dt7_dr16_data->rc.ch[3] = 0;
    dt7_dr16_data->rc.ch[4] = 0;
    dt7_dr16_data->rc.s[0] = 0;
    dt7_dr16_data->rc.s[1] = 0;
    dt7_dr16_data->mouse.x = 0;
    dt7_dr16_data->mouse.y = 0;
    dt7_dr16_data->mouse.z = 0;
    dt7_dr16_data->mouse.press_l = 0;
    dt7_dr16_data->mouse.press_r = 0;
    dt7_dr16_data->key.v = 0;
    return false;
}

#endif /* RFL_DEV_REMOTE_CONTROLL_DT7_DR16 */
