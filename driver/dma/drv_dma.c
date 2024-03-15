#include "stdlib.h"
#include "string.h"

#include "drv_dma.h"

static rfl_dma_s *dma_list = NULL;

void rflDmaInit(void)
{
    dma_list = (rfl_dma_s *)malloc(RFL_DMA_NUM * sizeof(rfl_dma_s));
    memset(dma_list, 0, RFL_DMA_NUM * sizeof(rfl_dma_s));
}

void rflDmaAddCallbackFunction(DMA_Type *dma, uint8_t ch_index, void (*callback_func)(void))
{
#if RFL_CONFIG_CORE == RFL_CORE_WPIE_HPM6750
    uint8_t dma_ordinal = (dma == HPM_HDMA ? HDMA : XDMA);
#elif RFL_CONFIG_CORE == RFL_CORE_RM_C_BORAD
    uint8_t dma_ordinal = 0;
#endif

    if (dma_list[dma_ordinal].used_channle_num >= RFL_DMA_CHANNEL_NUM)
        return;

    dma_list[dma_ordinal].used_channle_num++;
    dma_list[dma_ordinal].channle_table[dma_list[dma_ordinal].used_channle_num - 1] = ch_index;
    dma_list[dma_ordinal].callback_func_list[dma_list[dma_ordinal].used_channle_num - 1] = callback_func;
}

#if RFL_CONFIG_CORE == RFL_CORE_WPIE_HPM6750

void hdma_isr(void)
{
    volatile hpm_stat_t stat_chn;

    for (uint8_t i = 0; i < dma_list[0].used_channle_num; i++)
    {
        if (stat_chn = dma_check_transfer_status(BOARD_HDMA, dma_list[0].channle_table[i]),
            stat_chn & DMA_CHANNEL_STATUS_TC)
        {
            dma_list[0].callback_func_list[i]();
        }
    }
}
SDK_DECLARE_EXT_ISR_M(BOARD_HDMA_IRQ, hdma_isr)

void xdma_isr(void)
{
    volatile hpm_stat_t stat_chn;

    for (uint8_t i = 0; i < dma_list[1].used_channle_num; i++)
    {
        if (stat_chn = dma_check_transfer_status(BOARD_XDMA, dma_list[1].channle_table[i]),
            stat_chn & DMA_CHANNEL_STATUS_TC)
        {
            dma_list[1].callback_func_list[i]();
        }
    }
}
SDK_DECLARE_EXT_ISR_M(BOARD_XDMA_IRQ, xdma_isr)

#elif RFL_CONFIG_CORE == RFL_CORE_RM_C_BORAD

#endif
