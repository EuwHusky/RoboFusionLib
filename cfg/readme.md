# 请在这个文件夹下自行添加配置文件

## 文件名

rfl_config.h

## 文件内容

```C

#ifndef _RFL_CONFIG_H__
#define _RFL_CONFIG_H__

#include "rfl_core_rm_c_board.h"
#include "rfl_core_wpie_hpm6750.h"

/* 主控 ------------------------------ */

#define RFL_CONFIG_CORE RFL_CORE_WPIE_HPM6750

/* 遥控器模块配置 ------------------------------ */

#define RFL_DEV_REMOTE_CONTROLL_DT7_DR16 1 // 打算之后把遥控器做成抽象硬件模块，但目前仍为应用层模块

/* 电机模块配置 ------------------------------ */

#define RFL_BSP_RM_MOTOR_ENABLED 1      // 未完成开关功能，必须开启
#define RFL_BSP_UNITREE_MOTOR_ENABLED 0 // 已完成开关功能
#define RFL_BSP_DAMIAO_MOTOR_ENABLED 0  // 已完成开关功能

/* 编码器模块配置 ------------------------------ */

#define RFL_DEV_ENCODER_MA600 0          // 已完成开关功能
#define RFL_DEV_ENCODER_MA600_USED_NUM 1 // 使用的MA600个数

#endif /* _RFL_CONFIG_H__ */


```
