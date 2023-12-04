#ifndef _ALGO_AHRS_H__
#define _ALGO_AHRS_H__

// #include "QuaternionEKF.h"
#include "algo_ahrs_mahony.h"

void RflAhrsInit(ahrs_data_s_t *ahrs_data);

void RflAhrsUpdate(ahrs_data_s_t *ahrs_data, float gx, float gy, float gz, float ax, float ay, float az);

#endif /* _ALGO_AHRS_H__ */
