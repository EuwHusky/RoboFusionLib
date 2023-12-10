//=====================================================================================================
// MahonyAHRS.c
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

//---------------------------------------------------------------------------------------------------
// Header files

#include <math.h>

#include "algo_ahrs_mahony.h"

//---------------------------------------------------------------------------------------------------
// Function declarations

float invSqrt(float x);
volatile float q0, q1, q2, q3; // quaternion of sensor frame relative to auxiliary frame
//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

void MahonyAHRSupdate(ahrs_data_s_t *ahrs_data, float gx, float gy, float gz, float ax, float ay, float az, float mx,
                      float my, float mz)
{
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
    {
        MahonyAHRSupdateIMU(ahrs_data, gx, gy, gz, ax, ay, az);
        return;
    }

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = ahrs_data->quaternion[0] * ahrs_data->quaternion[0];
        q0q1 = ahrs_data->quaternion[0] * ahrs_data->quaternion[1];
        q0q2 = ahrs_data->quaternion[0] * ahrs_data->quaternion[2];
        q0q3 = ahrs_data->quaternion[0] * ahrs_data->quaternion[3];
        q1q1 = ahrs_data->quaternion[1] * ahrs_data->quaternion[1];
        q1q2 = ahrs_data->quaternion[1] * ahrs_data->quaternion[2];
        q1q3 = ahrs_data->quaternion[1] * ahrs_data->quaternion[3];
        q2q2 = ahrs_data->quaternion[2] * ahrs_data->quaternion[2];
        q2q3 = ahrs_data->quaternion[2] * ahrs_data->quaternion[3];
        q3q3 = ahrs_data->quaternion[3] * ahrs_data->quaternion[3];

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        // Estimated direction of gravity and magnetic field
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        // Error is sum of cross product between estimated direction and measured direction of field vectors
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

        // Compute and apply integral feedback if enabled
        if (ahrs_data->twoKi > 0.0f)
        {
            ahrs_data->integralFBx += ahrs_data->twoKi * halfex * (1.0f / sampleFreq); // integral error scaled by Ki
            ahrs_data->integralFBy += ahrs_data->twoKi * halfey * (1.0f / sampleFreq);
            ahrs_data->integralFBz += ahrs_data->twoKi * halfez * (1.0f / sampleFreq);
            gx += ahrs_data->integralFBx; // apply integral feedback
            gy += ahrs_data->integralFBy;
            gz += ahrs_data->integralFBz;
        }
        else
        {
            ahrs_data->integralFBx = 0.0f; // prevent integral windup
            ahrs_data->integralFBy = 0.0f;
            ahrs_data->integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += ahrs_data->twoKp * halfex;
        gy += ahrs_data->twoKp * halfey;
        gz += ahrs_data->twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * (1.0f / sampleFreq)); // pre-multiply common factors
    gy *= (0.5f * (1.0f / sampleFreq));
    gz *= (0.5f * (1.0f / sampleFreq));
    qa = ahrs_data->quaternion[0];
    qb = ahrs_data->quaternion[1];
    qc = ahrs_data->quaternion[2];
    ahrs_data->quaternion[0] += (-qb * gx - qc * gy - ahrs_data->quaternion[3] * gz);
    ahrs_data->quaternion[1] += (qa * gx + qc * gz - ahrs_data->quaternion[3] * gy);
    ahrs_data->quaternion[2] += (qa * gy - qb * gz + ahrs_data->quaternion[3] * gx);
    ahrs_data->quaternion[3] += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(
        ahrs_data->quaternion[0] * ahrs_data->quaternion[0] + ahrs_data->quaternion[1] * ahrs_data->quaternion[1] +
        ahrs_data->quaternion[2] * ahrs_data->quaternion[2] + ahrs_data->quaternion[3] * ahrs_data->quaternion[3]);
    ahrs_data->quaternion[0] *= recipNorm;
    ahrs_data->quaternion[1] *= recipNorm;
    ahrs_data->quaternion[2] *= recipNorm;
    ahrs_data->quaternion[3] *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MahonyAHRSupdateIMU(ahrs_data_s_t *ahrs_data, float gx, float gy, float gz, float ax, float ay, float az)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx =
            ahrs_data->quaternion[1] * ahrs_data->quaternion[3] - ahrs_data->quaternion[0] * ahrs_data->quaternion[2];
        halfvy =
            ahrs_data->quaternion[0] * ahrs_data->quaternion[1] + ahrs_data->quaternion[2] * ahrs_data->quaternion[3];
        halfvz = ahrs_data->quaternion[0] * ahrs_data->quaternion[0] - 0.5f +
                 ahrs_data->quaternion[3] * ahrs_data->quaternion[3];

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        if (ahrs_data->twoKi > 0.0f)
        {
            ahrs_data->integralFBx += ahrs_data->twoKi * halfex * (1.0f / sampleFreq); // integral error scaled by Ki
            ahrs_data->integralFBy += ahrs_data->twoKi * halfey * (1.0f / sampleFreq);
            ahrs_data->integralFBz += ahrs_data->twoKi * halfez * (1.0f / sampleFreq);
            gx += ahrs_data->integralFBx; // apply integral feedback
            gy += ahrs_data->integralFBy;
            gz += ahrs_data->integralFBz;
        }
        else
        {
            ahrs_data->integralFBx = 0.0f; // prevent integral windup
            ahrs_data->integralFBy = 0.0f;
            ahrs_data->integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += ahrs_data->twoKp * halfex;
        gy += ahrs_data->twoKp * halfey;
        gz += ahrs_data->twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * (1.0f / sampleFreq)); // pre-multiply common factors
    gy *= (0.5f * (1.0f / sampleFreq));
    gz *= (0.5f * (1.0f / sampleFreq));
    qa = ahrs_data->quaternion[0];
    qb = ahrs_data->quaternion[1];
    qc = ahrs_data->quaternion[2];
    ahrs_data->quaternion[0] += (-qb * gx - qc * gy - ahrs_data->quaternion[3] * gz);
    ahrs_data->quaternion[1] += (qa * gx + qc * gz - ahrs_data->quaternion[3] * gy);
    ahrs_data->quaternion[2] += (qa * gy - qb * gz + ahrs_data->quaternion[3] * gx);
    ahrs_data->quaternion[3] += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(
        ahrs_data->quaternion[0] * ahrs_data->quaternion[0] + ahrs_data->quaternion[1] * ahrs_data->quaternion[1] +
        ahrs_data->quaternion[2] * ahrs_data->quaternion[2] + ahrs_data->quaternion[3] * ahrs_data->quaternion[3]);
    ahrs_data->quaternion[0] *= recipNorm;
    ahrs_data->quaternion[1] *= recipNorm;
    ahrs_data->quaternion[2] *= recipNorm;
    ahrs_data->quaternion[3] *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

//====================================================================================================
// END OF CODE
//====================================================================================================
