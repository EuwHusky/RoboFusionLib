#ifndef _MT_TASK_H__
#define _MT_TASK_H__

#include "stdint.h"

// 速度环PID参数
#define RM_M2006_SPEED_PID_KP (2500.0f)
#define RM_M2006_SPEED_PID_KI (250.0f)
#define RM_M2006_SPEED_PID_KD (0.0f)
#define RM_M2006_SPEED_PID_MAX_IOUT (2500.0f)
#define RM_M2006_SPEED_PID_MAX_OUT (10000.0f)
// 角度环PID参数
#define RM_M2006_ANGLE_PID_KP (1.0f)
#define RM_M2006_ANGLE_PID_KI (0.0f)
#define RM_M2006_ANGLE_PID_KD (0.2f)
#define RM_M2006_ANGLE_PID_MAX_IOUT (0.0f)
#define RM_M2006_ANGLE_PID_MAX_OUT (32.0f)

// 速度环PID参数
#define RM_M3508_SPEED_PID_KP (2000.0f)
#define RM_M3508_SPEED_PID_KI (16.0f)
#define RM_M3508_SPEED_PID_KD (0.0f)
#define RM_M3508_SPEED_PID_MAX_IOUT (4000.0f)
#define RM_M3508_SPEED_PID_MAX_OUT (16000.0f)
// 角度环PID参数
#define RM_M3508_ANGLE_PID_KP (10.0f)
#define RM_M3508_ANGLE_PID_KI (0.0f)
#define RM_M3508_ANGLE_PID_KD (0.0f)
#define RM_M3508_ANGLE_PID_MAX_IOUT (0.0f)
#define RM_M3508_ANGLE_PID_MAX_OUT (8.0f)

extern void mt_task(void const *pvParameters);
extern void unitree_uart_init(void);

#endif /* _MT_TASK_H__ */
