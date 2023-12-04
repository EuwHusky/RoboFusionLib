#ifndef _DRV_DELAY_H__
#define _DRV_DELAY_H__

#include "stdint.h"

extern void delay_init(void);
extern void delay_us(uint16_t nus);
extern void delay_ms(uint16_t nms);

#endif /* _DRV_DELAY_H__ */
