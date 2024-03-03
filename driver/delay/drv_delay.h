#ifndef _DRV_DELAY_H__
#define _DRV_DELAY_H__

#include "stdint.h"

#include "rfl_config.h"

#if RFL_CONFIG_CORE == RFL_CORE_WPIE_HPM6750
#elif RFL_CONFIG_CORE == RFL_CORE_RM_C_BORAD
extern void delay_init(void);
extern void delay_us(uint16_t nus);
extern void delay_ms(uint16_t nms);
#endif

extern void rflOsDelayMs(uint16_t ms);

#endif /* _DRV_DELAY_H__ */
