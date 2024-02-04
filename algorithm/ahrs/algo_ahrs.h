#ifndef _ALGO_AHRS_H__
#define _ALGO_AHRS_H__

#include "algo_ahrs_config.h"

#if AHRS_USE == USE_MAHONY
#include "algo_ahrs_mahony.h"
#elif AHRS_USE == USE_MADGWICK
#include "algo_ahrs_madgwick.h"
#elif AHRS_USE == USE_QEKF
#include "algo_ahrs_quaternionekf.h"
#endif

void RflAhrsInit(ahrs_data_s_t *ahrs_data);

void RflAhrsUpdate(ahrs_data_s_t *ahrs_data, float gx, float gy, float gz, float ax, float ay, float az);

#endif /* _ALGO_AHRS_H__ */
