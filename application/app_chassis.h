#ifndef _APP_CHASSIS_H__
#define _APP_CHASSIS_H__

#include "app_chassis_config.h"

typedef struct RflChassis
{
    /* 基础参数 */
    rfl_chassis_type_e type;
    rfl_chassis_control_mode_e mode_;

    /* 控制量 */
    float set_vx; // 底盘速度 前进方向 前为正，单位 m/s
    float set_vy; // 底盘速度 左右方向 左为正  单位 m/s
    float set_wz; // 底盘旋转角速度，逆时针为正 单位 rad/s
} rfl_chassis_s;

#endif /* _APP_CHASSIS_H__ */
