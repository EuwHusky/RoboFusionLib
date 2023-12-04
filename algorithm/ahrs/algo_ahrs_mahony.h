//=====================================================================================================
// MahonyAHRS.h
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef _MahonyAHRS_H__
#define _MahonyAHRS_H__

//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq 1000.0f     // sample frequency in Hz
#define twoKpDef (2.0f * 0.5f) // 2 * proportional gain
#define twoKiDef (2.0f * 0.0f) // 2 * integral gain

/**
 * @brief 航姿参考系统数据
 */
typedef struct AhrsData
{
    volatile float yaw;
    volatile float pitch;
    volatile float roll;

    volatile float quaternion[4]; // quaternion of sensor frame relative to auxiliary frame

    volatile float twoKp;                                 // 2 * proportional gain (Kp)
    volatile float twoKi;                                 // 2 * integral gain (Ki)
    volatile float integralFBx, integralFBy, integralFBz; // integral error terms scaled by Ki
} ahrs_data_s_t;

//---------------------------------------------------------------------------------------------------
// Function declarations

void MahonyAHRSupdate(ahrs_data_s_t *ahrs_data, float gx, float gy, float gz, float ax, float ay, float az, float mx,
                      float my, float mz);
void MahonyAHRSupdateIMU(ahrs_data_s_t *ahrs_data, float gx, float gy, float gz, float ax, float ay, float az);

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
