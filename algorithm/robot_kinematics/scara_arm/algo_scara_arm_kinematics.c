#include "math.h"
#include "string.h"

#include "algo_scara_arm_kinematics.h"

void rflPose6DToTransformMatrix(rfl_matrix_instance *trans_mat, const float pose_6d[6])
{
    if (trans_mat->numCols != 4 || trans_mat->numRows != 4)
        return;

    float sin_angle_z = sinf(pose_6d[3]);
    float cos_angle_z = cosf(pose_6d[3]);
    float sin_angle_y = sinf(pose_6d[4]);
    float cos_angle_y = cosf(pose_6d[4]);
    float sin_angle_x = sinf(pose_6d[5]);
    float cos_angle_x = cosf(pose_6d[5]);

    trans_mat->pData[0] = cos_angle_z * cos_angle_y;
    trans_mat->pData[1] = cos_angle_z * sin_angle_y * sin_angle_x - sin_angle_z * cos_angle_x;
    trans_mat->pData[2] = cos_angle_z * sin_angle_y * cos_angle_x + sin_angle_z * sin_angle_x;
    trans_mat->pData[3] = pose_6d[0];

    trans_mat->pData[4] = sin_angle_z * cos_angle_y;
    trans_mat->pData[5] = sin_angle_z * sin_angle_y * sin_angle_x + cos_angle_z * cos_angle_x;
    trans_mat->pData[6] = sin_angle_z * sin_angle_y * cos_angle_x - cos_angle_z * sin_angle_x;
    trans_mat->pData[7] = pose_6d[1];

    trans_mat->pData[8] = -sin_angle_y;
    trans_mat->pData[9] = cos_angle_y * sin_angle_x;
    trans_mat->pData[10] = cos_angle_y * cos_angle_x;
    trans_mat->pData[11] = pose_6d[2];

    trans_mat->pData[12] = 0.0f;
    trans_mat->pData[13] = 0.0f;
    trans_mat->pData[14] = 0.0f;
    trans_mat->pData[15] = 1.0f;
}

static void calc_joint_dh_to_transform_matrix(rfl_matrix_instance *trans_mat, const float joint_dh[5])
{
    if (trans_mat->numCols != 4 || trans_mat->numRows != 4)
        return;

    float sin_alpha = sinf(joint_dh[0]);
    float cos_alpha = cosf(joint_dh[0]);
    float sin_theta = sinf(joint_dh[3] - joint_dh[4]);
    float cos_theta = cosf(joint_dh[3] - joint_dh[4]);

    trans_mat->pData[0] = cos_theta;
    trans_mat->pData[1] = -sin_theta;
    trans_mat->pData[2] = 0;
    trans_mat->pData[3] = joint_dh[1];

    trans_mat->pData[4] = sin_theta * cos_alpha;
    trans_mat->pData[5] = cos_theta * cos_alpha;
    trans_mat->pData[6] = -sin_alpha;
    trans_mat->pData[7] = -sin_alpha * joint_dh[2];

    trans_mat->pData[8] = sin_theta * sin_alpha;
    trans_mat->pData[9] = cos_theta * sin_alpha;
    trans_mat->pData[10] = cos_alpha;
    trans_mat->pData[11] = cos_alpha * joint_dh[2];

    trans_mat->pData[12] = 0.0f;
    trans_mat->pData[13] = 0.0f;
    trans_mat->pData[14] = 0.0f;
    trans_mat->pData[15] = 1.0f;
}

static void calc_tool_to_base_transform_matrix(rfl_scara_arm_s *scara_arm)
{
    // 缓存

    rfl_matrix_instance calc_temp_mat_1;
    float calc_temp_mat_1_data[16] = {0.0f};
    rflMatrixInit(&calc_temp_mat_1, 4, 4, calc_temp_mat_1_data);
    rfl_matrix_instance calc_temp_mat_2;
    float calc_temp_mat_2_data[16] = {0.0f};
    rflMatrixInit(&calc_temp_mat_2, 4, 4, calc_temp_mat_2_data);
    rfl_matrix_instance calc_temp_mat_3;
    float calc_temp_mat_3_data[16] = {0.0f};
    rflMatrixInit(&calc_temp_mat_3, 4, 4, calc_temp_mat_3_data);

    // 计算

    memcpy(calc_temp_mat_3_data, scara_arm->j1_to_base_tmat.pData, 16 * sizeof(float)); // j0 to base

    calc_joint_dh_to_transform_matrix(&calc_temp_mat_1, scara_arm->dh[0]); // dh 1

    rflMatrixMult(&calc_temp_mat_3, &calc_temp_mat_1, &calc_temp_mat_2); // j1 to base

    calc_joint_dh_to_transform_matrix(&calc_temp_mat_1, scara_arm->dh[1]); // dh 2

    rflMatrixMult(&calc_temp_mat_2, &calc_temp_mat_1, &calc_temp_mat_3); // j2 to base

    calc_joint_dh_to_transform_matrix(&calc_temp_mat_1, scara_arm->dh[2]); // dh 3

    rflMatrixMult(&calc_temp_mat_3, &calc_temp_mat_1, &calc_temp_mat_2); // j3 to base

    calc_joint_dh_to_transform_matrix(&calc_temp_mat_1, scara_arm->dh[3]); // dh 4

    rflMatrixMult(&calc_temp_mat_2, &calc_temp_mat_1, &calc_temp_mat_3); // j4 to base

    calc_joint_dh_to_transform_matrix(&calc_temp_mat_1, scara_arm->dh[4]); // dh 5

    rflMatrixMult(&calc_temp_mat_3, &calc_temp_mat_1, &calc_temp_mat_2); // j5 to base

    calc_joint_dh_to_transform_matrix(&calc_temp_mat_1, scara_arm->dh[5]); // dh 6

    rflMatrixMult(&calc_temp_mat_2, &calc_temp_mat_1, &calc_temp_mat_3); // j6 to base

    rflMatrixMult(&calc_temp_mat_3, &scara_arm->tool_to_j6_tmat, &scara_arm->tool_to_base_tmat); // tool to base
}

void rflScaraSolveForwardKinematics(rfl_scara_arm_s *scara_arm)
{
    calc_tool_to_base_transform_matrix(scara_arm);

    scara_arm->pose_6d[0] = scara_arm->tool_to_base_tmat.pData[3];
    scara_arm->pose_6d[1] = scara_arm->tool_to_base_tmat.pData[7];
    scara_arm->pose_6d[2] = scara_arm->tool_to_base_tmat.pData[11];
    scara_arm->pose_6d[3] = scara_arm->joints_value[1] + scara_arm->joints_value[2] + scara_arm->joints_value[3];
    scara_arm->pose_6d[4] = scara_arm->joints_value[4];
    scara_arm->pose_6d[5] = scara_arm->joints_value[5];
}

void rflScaraSolveInverseKinematics(rfl_scara_arm_s *scara_arm)
{
}
