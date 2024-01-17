#include "stdlib.h"
#include "string.h"

#include "algo_scara_arm.h"
#include "algo_scara_arm_kinematics.h"

void rflScaraArmGetDefaultConfig(rfl_scara_arm_config_s *config)
{
    memset(config, 0, sizeof(rfl_scara_arm_config_s));

    config->j1_to_base_pose_6d[0] = RFL_SCARA_ARM_JI_TO_BASE_DEFAULT_X;
    config->j1_to_base_pose_6d[1] = RFL_SCARA_ARM_JI_TO_BASE_DEFAULT_Y;
    config->j1_to_base_pose_6d[2] = RFL_SCARA_ARM_JI_TO_BASE_DEFAULT_Z;
    config->j1_to_base_pose_6d[3] = RFL_SCARA_ARM_JI_TO_BASE_DEFAULT_ANGLE_Z;
    config->j1_to_base_pose_6d[4] = RFL_SCARA_ARM_JI_TO_BASE_DEFAULT_ANGLE_Y;
    config->j1_to_base_pose_6d[5] = RFL_SCARA_ARM_JI_TO_BASE_DEFAULT_ANGLE_X;

    config->tool_to_j6_pose_6d[0] = RFL_SCARA_ARM_TOOL_TO_J6_DEFAULT_X;
    config->tool_to_j6_pose_6d[1] = RFL_SCARA_ARM_TOOL_TO_J6_DEFAULT_Y;
    config->tool_to_j6_pose_6d[2] = RFL_SCARA_ARM_TOOL_TO_J6_DEFAULT_Z;
    config->tool_to_j6_pose_6d[3] = RFL_SCARA_ARM_TOOL_TO_J6_DEFAULT_ANGLE_Z;
    config->tool_to_j6_pose_6d[4] = RFL_SCARA_ARM_TOOL_TO_J6_DEFAULT_ANGLE_Y;
    config->tool_to_j6_pose_6d[5] = RFL_SCARA_ARM_TOOL_TO_J6_DEFAULT_ANGLE_X;

    config->dh[0][0] = RFL_SCARA_ARM_DEFAULT_JOINT_1_DH_ALPHA;
    config->dh[0][1] = RFL_SCARA_ARM_DEFAULT_JOINT_1_DH_A;
    config->dh[0][2] = RFL_SCARA_ARM_DEFAULT_JOINT_1_DH_D;
    config->dh[0][3] = RFL_SCARA_ARM_DEFAULT_JOINT_1_DH_THETA;
    config->dh[0][4] = RFL_SCARA_ARM_DEFAULT_JOINT_1_DH_THETA_OFFSET;

    config->dh[1][0] = RFL_SCARA_ARM_DEFAULT_JOINT_2_DH_ALPHA;
    config->dh[1][1] = RFL_SCARA_ARM_DEFAULT_JOINT_2_DH_A;
    config->dh[1][2] = RFL_SCARA_ARM_DEFAULT_JOINT_2_DH_D;
    config->dh[1][3] = RFL_SCARA_ARM_DEFAULT_JOINT_2_DH_THETA;
    config->dh[1][4] = RFL_SCARA_ARM_DEFAULT_JOINT_2_DH_THETA_OFFSET;

    config->dh[2][0] = RFL_SCARA_ARM_DEFAULT_JOINT_3_DH_ALPHA;
    config->dh[2][1] = RFL_SCARA_ARM_DEFAULT_JOINT_3_DH_A;
    config->dh[2][2] = RFL_SCARA_ARM_DEFAULT_JOINT_3_DH_D;
    config->dh[2][3] = RFL_SCARA_ARM_DEFAULT_JOINT_3_DH_THETA;
    config->dh[2][4] = RFL_SCARA_ARM_DEFAULT_JOINT_3_DH_THETA_OFFSET;

    config->dh[3][0] = RFL_SCARA_ARM_DEFAULT_JOINT_4_DH_ALPHA;
    config->dh[3][1] = RFL_SCARA_ARM_DEFAULT_JOINT_4_DH_A;
    config->dh[3][2] = RFL_SCARA_ARM_DEFAULT_JOINT_4_DH_D;
    config->dh[3][3] = RFL_SCARA_ARM_DEFAULT_JOINT_4_DH_THETA;
    config->dh[3][4] = RFL_SCARA_ARM_DEFAULT_JOINT_4_DH_THETA_OFFSET;

    config->dh[4][0] = RFL_SCARA_ARM_DEFAULT_JOINT_5_DH_ALPHA;
    config->dh[4][1] = RFL_SCARA_ARM_DEFAULT_JOINT_5_DH_A;
    config->dh[4][2] = RFL_SCARA_ARM_DEFAULT_JOINT_5_DH_D;
    config->dh[4][3] = RFL_SCARA_ARM_DEFAULT_JOINT_5_DH_THETA;
    config->dh[4][4] = RFL_SCARA_ARM_DEFAULT_JOINT_5_DH_THETA_OFFSET;

    config->dh[5][0] = RFL_SCARA_ARM_DEFAULT_JOINT_6_DH_ALPHA;
    config->dh[5][1] = RFL_SCARA_ARM_DEFAULT_JOINT_6_DH_A;
    config->dh[5][2] = RFL_SCARA_ARM_DEFAULT_JOINT_6_DH_D;
    config->dh[5][3] = RFL_SCARA_ARM_DEFAULT_JOINT_6_DH_THETA;
    config->dh[5][4] = RFL_SCARA_ARM_DEFAULT_JOINT_6_DH_THETA_OFFSET;

    // motors_config_builder(config);
}

void rflScaraArmInit(rfl_scara_arm_s *scara_arm, rfl_scara_arm_config_s *config)
{
    memset(scara_arm, 0, sizeof(rfl_scara_arm_s));

    /* 基础参数 */
    memset(&scara_arm->j1_to_base_tmat_data, 0, 16 * sizeof(float));
    rflMatrixInit(&scara_arm->j1_to_base_tmat, 4, 4, scara_arm->j1_to_base_tmat_data);
    rflPose6DToTransformMatrix(&scara_arm->j1_to_base_tmat, config->j1_to_base_pose_6d);
    memset(&scara_arm->tool_to_j6_tmat_data, 0, 16 * sizeof(float));
    rflMatrixInit(&scara_arm->tool_to_j6_tmat, 4, 4, scara_arm->tool_to_j6_tmat_data);
    rflPose6DToTransformMatrix(&scara_arm->tool_to_j6_tmat, config->tool_to_j6_pose_6d);

    memcpy(scara_arm->dh, config->dh, 6 * 5 * sizeof(float));

    /* 状态量 */
    memset(&scara_arm->tool_to_base_tmat_data, 0, 16 * sizeof(float));
    rflMatrixInit(&scara_arm->tool_to_base_tmat, 4, 4, scara_arm->tool_to_base_tmat_data);
}

void rflScaraArmUpdatePose6d(rfl_scara_arm_s *scara_arm, const float joints_value[6])
{
    for (uint8_t i = 0; i < 6; i++)
        scara_arm->joints_value[i] = joints_value[i];

    scara_arm->dh[0][2] = scara_arm->joints_value[0]; // Joint 1 distance
    scara_arm->dh[1][3] = scara_arm->joints_value[1]; // Joint 2 angle
    scara_arm->dh[2][3] = scara_arm->joints_value[2]; // Joint 3 angle
    scara_arm->dh[3][3] = scara_arm->joints_value[3]; // Joint 4 angle
    scara_arm->dh[4][3] = scara_arm->joints_value[4]; // Joint 5 angle
    scara_arm->dh[5][3] = scara_arm->joints_value[5]; // Joint 6 angle

    rflScaraSolveForwardKinematics(scara_arm);
}
