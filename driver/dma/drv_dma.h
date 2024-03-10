#ifndef _DRV_DMA_H__
#define _DRV_DMA_H__

#include "stdint.h"

#include "rfl_config.h"

#if RFL_CONFIG_CORE == RFL_CORE_WPIE_HPM6750

#define RFL_DMA_NUM RFL_CORE_WPIE_HPM6750_DMA_NUM
#define RFL_DMA_CHANNEL_NUM RFL_CORE_WPIE_HPM6750_DMACHN_NUM

typedef enum RflHpmDma
{
    HDMA = 0,
    XDMA,
} rfl_hpm_dma_e;

#elif RFL_CONFIG_CORE == RFL_CORE_RM_C_BORAD
#define RFL_DMA_NUM 1
#define RFL_DMA_CHANNEL_NUM 1

#endif

typedef struct RflDma
{
    uint8_t used_channle_num;                              /* 已使用的DMA通道数 */
    uint32_t channle_table[RFL_DMA_CHANNEL_NUM];           /* 已添加的channle */
    void (*callback_func_list[RFL_DMA_CHANNEL_NUM])(void); /* 回调函数 */
} rfl_dma_s;

#endif /* _DRV_DMA_H__ */
