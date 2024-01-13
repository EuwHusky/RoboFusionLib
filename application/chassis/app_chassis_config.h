#ifndef _APP_CHASSIS_CONFIG_H__
#define _APP_CHASSIS_CONFIG_H__

#include "stdbool.h"
#include "stdint.h"

#include "dev_motor.h"

#define CHASSIS_MECANUM_MOTOR_NUM (4U)
#define CHASSIS_COAXIAL_MECANUM_MOTOR_NUM (4U)
#define CHASSIS_OMNI_MOTOR_NUM (4U)
#define CHASSIS_DUAL_STEER_MOTOR_NUM (4U)
#define CHASSIS_FOUR_STEER_MOTOR_NUM (8U)

typedef enum RflChassisType
{
    RFL_CHASSIS_MECANUM = 0,     // 麦克纳姆轮底盘
    RFL_CHASSIS_COAXIAL_MECANUM, // 共轴麦克纳姆轮底盘
    RFL_CHASSIS_OMNI,            // 全向轮底盘
    RFL_CHASSIS_DUAL_STEER,      // 两舵轮两全向轮底盘
    RFL_CHASSIS_FOUR_STEER,      // 四舵轮底盘
} rfl_chassis_type_e;

typedef enum RflChassisControllerType
{
    RFL_CHASSIS_CONTROLLER_NONE = 0,        // 无控制器
    RFL_CHASSIS_CONTROLLER_PID,             // PID控制器
    RFL_CHASSIS_CONTROLLER_FEEDFORWARD_PID, // 前馈+PID控制器
} rfl_chassis_controller_type_e;

typedef enum RflChassisControlMode
{
    RFL_CHASSIS_CONTROL_MODE_NO_FORCE = 0, // 底盘无力
    RFL_CHASSIS_CONTROL_MODE_FOLLOW,       // 底盘整体跟随设定角度
    RFL_CHASSIS_CONTROL_MODE_STEER_FOLLOW, // 底盘整体不动，仅舵向电机跟随设定角度
    RFL_CHASSIS_CONTROL_MODE_SPIN,         // 底盘小陀螺
} rfl_chassis_control_mode_e;

typedef struct RflChassisMecanumParameter
{
    float length; // 四个轮子与地面的接触点形成的矩形平行于X方向的边长 单位 米（m）
    float width;  // 四个轮子与地面的接触点形成的矩形平行于Y方向的边长 单位 米（m）
} rfl_chassis_mecanum_parameter_s;

typedef struct RflChassisCoaxialMecanumParameter
{
    uint8_t mecanum_polarity[4];
    float inner_radius; // 内圈两个轮子与地面的接触点到底盘旋转中心的距离 单位 米（m）
    float outer_radius; // 外圈两个轮子与地面的接触点到底盘旋转中心的距离 单位 米（m）
} rfl_chassis_coaxial_mecanum_parameter_s;

typedef struct RflChassisOmniParameter
{
    float length; // 轮子与地面的接触点到底盘旋转中心的距离 单位 米（m）
} rfl_chassis_omni_parameter_s;

typedef struct RflChassisDualSteerParameter
{
    uint8_t motor_polarity[6];
    bool is_steer_motor[6];
    float length; // 轮子与地面的接触点到底盘旋转中心的距离 单位 米（m）
} rfl_chassis_dual_steer_parameter_s;

typedef struct RflChassisFourSteerParameter
{
    uint8_t motor_polarity[8];
    bool is_steer_motor[8];
    float length; // 轮子与地面的接触点到底盘旋转中心的距离 单位 米（m）
} rfl_chassis_four_steer_parameter_s;

typedef struct RflChassisMotorConfig
{
    rfl_motor_type_e motor_type;
    float effector_transmission_ratio; // 电机末端执行器转一圈时电机转子转过的圈数

    const rfl_angle_s *external_angle;

    float angle_pid_param[5];
    float speed_pid_param[5];

    uint8_t can_ordinal;
    uint32_t can_id;
} rfl_chassis_motor_config_s;

typedef struct RflChassisConfig
{
    /* 底盘配置 */
    rfl_chassis_type_e type;
    rfl_chassis_control_mode_e mode;
    float wheel_radius;

    // AB麦轮标志，以底盘正方向为前，0代表该轮与地面接触的辊子为左前-右后方向，非0代表该轮与地面接触的辊子为右前-左后方向
    uint8_t mecanum_polarity[4]; // 数组大小为了方便而设为了可能会用到的最大值
    // 电机安装方位标志，俯视底盘并以底盘正方向为上，数组中1说明电机安装在轮子左侧，0说明电机安装在轮子右侧
    uint8_t motor_polarity[8]; // 数组大小为了方便而设为了可能会用到的最大值
    bool is_steer_motor[8];    // 数组大小为了方便而设为了可能会用到的最大值

    float width;
    float length;
    float inner_radius;
    float outer_radius;

    /* 控制量 */
    rfl_chassis_controller_type_e controller_type;
    float angle_pid_param[5];

    /* 状态量 */
    rfl_angle_s angle_offset;
    const rfl_angle_s *external_angle;

    /* 设备配置 */
    uint8_t motor_num;
    rfl_chassis_motor_config_s *motor_config_list;
} rfl_chassis_config_s;

// 默认车轮半径
#define RFL_CHASSIS_DEFAULT_MECANUM_WHEEL_RADIUS (0.076987f)
#define RFL_CHASSIS_DEFAULT_OMNI_WHEEL_RADIUS (0.07625f)
#define RFL_CHASSIS_DEFAULT_DRIVING_WHEEL_RADIUS (0.07625f)

#define RFL_CHASSIS_MECANUM_DEFAULT_LENGTH (0.56f)
#define RFL_CHASSIS_MECANUM_DEFAULT_WIDTH (0.56f)
#define RFL_CHASSIS_COAXIAL_MECANUM_DEFAULT_INNER_RADIUS (0.5f)
#define RFL_CHASSIS_COAXIAL_MECANUM_DEFAULT_OUTER_RADIUS (0.6f)
#define RFL_CHASSIS_OMNI_DEFAULT_LENGTH (0.56f)
#define RFL_CHASSIS_STEER_DEFAULT_LENGTH (0.56f)

#define RFL_CHASSIS_DEFAULT_ANGLE_PID_KP (0.0f)
#define RFL_CHASSIS_DEFAULT_ANGLE_PID_KI (0.0f)
#define RFL_CHASSIS_DEFAULT_ANGLE_PID_KD (0.0f)
#define RFL_CHASSIS_DEFAULT_ANGLE_PID_MAX_IOUT (0.0f)
#define RFL_CHASSIS_DEFAULT_ANGLE_PID_MAX_OUT (0.0f)

#define RFL_CHASSIS_DEFAULT_DRIVING_RM_M3508_SPEED_PID_KP (1200.0f)
#define RFL_CHASSIS_DEFAULT_DRIVING_RM_M3508_SPEED_PID_KI (12.0f)
#define RFL_CHASSIS_DEFAULT_DRIVING_RM_M3508_SPEED_PID_KD (0.0f)
#define RFL_CHASSIS_DEFAULT_DRIVING_RM_M3508_SPEED_PID_MAX_IOUT (4000.0f)
#define RFL_CHASSIS_DEFAULT_DRIVING_RM_M3508_SPEED_PID_MAX_OUT (16000.0f)

#define RFL_CHASSIS_DEFAULT_STEER_RM_M3508_SPEED_PID_KP (0.0f)
#define RFL_CHASSIS_DEFAULT_STEER_RM_M3508_SPEED_PID_KI (0.0f)
#define RFL_CHASSIS_DEFAULT_STEER_RM_M3508_SPEED_PID_KD (0.0f)
#define RFL_CHASSIS_DEFAULT_STEER_RM_M3508_SPEED_PID_MAX_IOUT (0.0f)
#define RFL_CHASSIS_DEFAULT_STEER_RM_M3508_SPEED_PID_MAX_OUT (0.0f)

#define RFL_CHASSIS_DEFAULT_STEER_RM_M3508_ANGLE_PID_KP (0.0f)
#define RFL_CHASSIS_DEFAULT_STEER_RM_M3508_ANGLE_PID_KI (0.0f)
#define RFL_CHASSIS_DEFAULT_STEER_RM_M3508_ANGLE_PID_KD (0.0f)
#define RFL_CHASSIS_DEFAULT_STEER_RM_M3508_ANGLE_PID_MAX_IOUT (0.0f)
#define RFL_CHASSIS_DEFAULT_STEER_RM_M3508_ANGLE_PID_MAX_OUT (0.0f)

#endif /* _APP_CHASSIS_CONFIG_H__ */
