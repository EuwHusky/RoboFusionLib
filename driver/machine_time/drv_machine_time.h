#ifndef CNU_HPM_BSP_CYC_H
#define CNU_HPM_BSP_CYC_H

#include "rfl_config.h"

#if (RFL_CONFIG_CORE == RFL_CORE_WPIE_HPM6750)

#include "stdint.h"

typedef struct
{
    uint32_t s;
    uint16_t ms;
    uint16_t us;
} DWT_Time_t;

extern float GetDeltaT(uint32_t *cnt_last);
extern void DWT_SysTimeUpdate(void);
extern float DWT_GetTimeline_s(void);
extern void DWT_Delay(float Delay);

#endif /* RFL_CONFIG_CORE == RFL_CONFIG_CORE_HPM67XX */

#endif
