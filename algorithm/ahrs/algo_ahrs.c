#include "algo_ahrs.h"

#include <math.h>

static void get_angle(float *q, float *yaw, float *pitch, float *roll);

void RflAhrsInit(ahrs_data_s_t *ahrs_data)
{
    ahrs_data->quaternion[0] = 1.0f; // quaternion of sensor frame relative to auxiliary frame
    ahrs_data->quaternion[1] = 0.0f;
    ahrs_data->quaternion[2] = 0.0f;
    ahrs_data->quaternion[3] = 0.0f;

    ahrs_data->twoKp = twoKpDef; // 2 * proportional gain (Kp)
    ahrs_data->twoKi = twoKiDef; // 2 * integral gain (Ki)

    ahrs_data->integralFBx = 0.0f; // integral error terms scaled by Ki
    ahrs_data->integralFBy = 0.0f;
    ahrs_data->integralFBz = 0.0f;
}

void RflAhrsUpdate(ahrs_data_s_t *ahrs_data, float gx, float gy, float gz, float ax, float ay, float az)
{
    MahonyAHRSupdate(ahrs_data, gx, gy, gz, ax, ay, az, 0.0f, 0.0f, 0.0f);

    get_angle((float *)ahrs_data->quaternion, (float *)&ahrs_data->yaw, (float *)&ahrs_data->pitch,
              (float *)&ahrs_data->roll);
}

static void get_angle(float *q, float *yaw, float *pitch, float *roll)
{
    *yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);
    *pitch = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2]));
    *roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f);
}
