#ifndef _APP_CHASSIS_H__
#define _APP_CHASSIS_H__

#include "app_chassis_config.h"

typedef struct RflChassis
{
    /* 基础参数 */
    rfl_chassis_type_e type;
    rfl_chassis_control_mode_e mode_;
    float wheel_radius; // 车轮半径 单位 米（m）

    void *parameter; // 底盘参数

    /* 控制量 */
    float set_vx_; // 设定的底盘速度 前进方向 前为正 单位 米每秒（m/s）
    float set_vy_; // 设定的底盘速度 左右方向 左为正 单位 米每秒（m/s）
    float set_wz_; // 设定的底盘旋转角速度 逆时针为正 单位 弧度每秒（rad/s）

    void *controller; // 底盘控制器

    /* 状态量 */
    float vx_; // 底盘速度 前进方向 前为正 单位 米每秒（m/s）
    float vy_; // 底盘速度 左右方向 左为正 单位 米每秒（m/s）
    float wz_; // 底盘旋转角速度 逆时针为正 单位 弧度每秒（rad/s）

    rfl_angle_s angle_offset;
    const rfl_angle_s *external_angle;
    rfl_angle_s angle_; //

    /* 设备 */
    uint8_t motor_num;
    // 底盘电机组，数组大小代表电机数量，数组元素代表电机型号，数组下标代表电机顺序
    // 电机顺序从0开始，假设底盘向其正方向有一射线，并向着逆时针方向扫掠，扫掠到的电机次序作为电机顺序
    // 若是舵轮则优先记录舵向电机，驱动电机在舵向电机之后
    rfl_motor_s *motors;
} rfl_chassis_s;

extern void rflChassisGetDefaultConfig(rfl_chassis_config_s *chassis_config, rfl_chassis_type_e type);
extern void rflChassisInit(rfl_chassis_s *chassis, rfl_chassis_config_s *chassis_config);
extern void rflChassisUpdate(rfl_chassis_s *chassis);

extern void rflChassisSetAngleOffset(rfl_chassis_s *chassis, rfl_angle_s angle_offset);
extern void rflChassisSetMode(rfl_chassis_s *chassis, rfl_chassis_control_mode_e mode);

#endif /* _APP_CHASSIS_H__ */
