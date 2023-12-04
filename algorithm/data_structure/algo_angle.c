#include "algo_angle.h"

void rflAngleUpdate(rfl_angle_s *angle, rfl_angle_format_e angle_format, float source_data)
{
    if (angle == NULL)
        return;

    if (angle_format == RFL_ANGLE_FORMAT_RADIAN)
    {
        angle->rad = source_data;
        angle->deg = angle->rad * RADIAN_TO_DEGREE_FACTOR;
    }
    else if (angle_format == RFL_ANGLE_FORMAT_DEGREE)
    {
        angle->deg = source_data;
        angle->rad = angle->deg * DEGREE_TO_RADIAN_FACTOR;
    }
}
