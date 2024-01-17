#ifndef _ALGO_ANGLE_H__
#define _ALGO_ANGLE_H__

#include "stddef.h"
#include "stdint.h"

#ifndef RAD_PI
#define RAD_PI (3.1415926535897932384626433832795f)
#endif
#ifndef RAD_2_PI
#define RAD_2_PI (6.283185307179586476925286766559f)
#endif
#ifndef DEG_PI
#define DEG_PI (180.0f)
#endif

#ifndef RADIAN_TO_DEGREE_FACTOR
#define RADIAN_TO_DEGREE_FACTOR (57.295779513082320876798154814105f)
#endif
#ifndef DEGREE_TO_RADIAN_FACTOR
#define DEGREE_TO_RADIAN_FACTOR (0.01745329251994329576923690768489f)
#endif

/**
 * @brief 角度单位
 */
typedef enum RflAngleFormat
{
    RFL_ANGLE_FORMAT_DEGREE = 0, // 角度制
    RFL_ANGLE_FORMAT_RADIAN      // 弧度制
} rfl_angle_format_e;

/**
 * @brief 角度
 */
typedef struct RflAngle
{
    float rad;
    float deg;
} rfl_angle_s;

extern void rflAngleUpdate(rfl_angle_s *angle, rfl_angle_format_e format, float source);

#endif /* _ALGO_ANGLE_H__ */
