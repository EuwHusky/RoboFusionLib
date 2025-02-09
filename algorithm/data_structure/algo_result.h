#ifndef _ALGO_RESULT_H__
#define _ALGO_RESULT_H__

#include "stdint.h"

typedef enum
{
    RFL_SUCCESS = 0,
    RFL_ERROR_INVALID_ARG,
    RFL_ERROR_NULL_POINTER,
    RFL_ERROR_OUT_OF_RANGE,
    RFL_ERROR_OUT_OF_MEMORY,

} RflError;

typedef struct
{
    union {
        uint32_t u;
        int32_t i;
        float f;
    } value;
    RflError error;

} RflResult;

#endif /* _ALGO_RESULT_H__ */
