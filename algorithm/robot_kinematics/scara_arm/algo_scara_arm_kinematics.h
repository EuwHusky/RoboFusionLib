#ifndef _APP_SCARA_ARM_KINEMATICS_H__
#define _APP_SCARA_ARM_KINEMATICS_H__

#include "stdbool.h"
#include "stdint.h"

#include "algo_scara_arm.h"

#include "algo_matrix.h"

extern void rflPose6DToTransformMatrix(rfl_matrix_instance *trans_mat, const float pose_6d[6]);

extern void rflScaraSolveForwardKinematics(rfl_scara_arm_s *scara_arm);

#endif /* _APP_SCARA_ARM_KINEMATICS_H__ */
