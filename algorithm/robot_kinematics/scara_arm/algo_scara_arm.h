#ifndef _APP_SCARA_ARM_H__
#define _APP_SCARA_ARM_H__

#include "algo_scara_arm_config.h"

#include "algo_matrix.h"

#include "dev_motor.h"

typedef struct RflScaraArm
{
    /* 基础参数 */

    rfl_matrix_instance j1_to_base_tmat; // 关节1坐标系到基坐标系变换矩阵
    float j1_to_base_tmat_data[16];
    rfl_matrix_instance tool_to_j6_tmat; // 工具坐标系到关节6坐标系变换矩阵
    float tool_to_j6_tmat_data[16];

    /**
     * @brief   Scara构型机械臂DH参数
     * [0:5]    Joint 1-6
     * [x][0]   alpha
     * [x][1]   a
     * [x][2]   d
     * [x][3]   theta
     * [x][4]   theta_offset
     */
    float dh[6][5];

    /* 状态量 */

    /**
     * @brief   关节变量 距离单位 米（m） 角度单位 弧度（rad）
     * [0:5]    J1-distance J2-angle J3-angle J4-angle J5-angle J6-angle
     */
    float joints_value[6];

    rfl_matrix_instance tool_to_base_tmat; // 工具坐标系到基坐标系变换矩阵
    float tool_to_base_tmat_data[16];

    /**
     * @brief   工具坐标系在基坐标系六自由度位姿 距离单位 米（m） 角度单位 弧度（rad）
     * [0:5]    X Y Z YAW PITCH ROLL
     */
    float pose_6d[6];

    /* 控制量 */

    /**
     * @brief   预期的工具坐标系在基坐标系六自由度位姿 距离单位 米（m） 角度单位 弧度（rad）
     * [0:5]    X Y Z YAW PITCH ROLL
     */
    float set_pose_6d[6];

    /**
     * @brief   逆运动学解算得到的期望关节变量 距离单位 米（m） 角度单位 弧度（rad）
     * [0:5]    J1-distance J2-angle J3-angle J4-angle J5-angle J6-angle
     */
    float set_joints_value[6];

    /* 设备 */
    rfl_motor_s *motors;
} rfl_scara_arm_s;

extern void rflScaraArmGetDefaultConfig(rfl_scara_arm_config_s *config);
extern void rflScaraArmInit(rfl_scara_arm_s *scara_arm, rfl_scara_arm_config_s *config);
extern void rflScaraArmUpdatePose6d(rfl_scara_arm_s *scara_arm, const float joints_value[6]);

#endif /* _APP_SCARA_ARM_H__ */
