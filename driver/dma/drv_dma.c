#include "stdlib.h"
#include "string.h"

#include "drv_dma.h"

#if RFL_CONFIG_CORE == RFL_CORE_WPIE_HPM6750
#include "board.h"
#include "hpm_dma_drv.h"
#include "hpm_dmamux_drv.h"

#elif RFL_CONFIG_CORE == RFL_CORE_RM_C_BORAD

#endif

static rfl_dma_s *dma_list = NULL;

void rflDmaInit(void)
{
    dma_list = (rfl_dma_s *)malloc(RFL_DMA_NUM * sizeof(rfl_dma_s));
    memset(dma_list, 0, RFL_DMA_NUM * sizeof(rfl_dma_s));
}

void rflDmaAddCallbackFunction(uint64_t dma, void (*callback_func)(void), uint32_t channle)
{
#if RFL_CONFIG_CORE == RFL_CORE_WPIE_HPM6750
    uint8_t dma_ordinal = (dma == HPM_HDMA ? HDMA : XDMA);
#elif RFL_CONFIG_CORE == RFL_CORE_RM_C_BORAD
    uint8_t dma_ordinal = 0;
#endif

    if (dma_list[dma_ordinal].used_channle_num >= RFL_DMA_CHANNEL_NUM)
        return;

    dma_list[dma_ordinal].used_channle_num++;
    dma_list[dma_ordinal].channle_table[dma_list[dma_ordinal].used_channle_num - 1] = channle;
    dma_list[dma_ordinal].callback_func_list[dma_list[dma_ordinal].used_channle_num - 1] = callback_func;
}

#if RFL_CONFIG_CORE == RFL_CORE_WPIE_HPM6750
void HDma_Callback(void)
{
    volatile hpm_stat_t stat_rx_chn;

    for (uint8_t i = 0; i < RFL_DMA_CHANNEL_NUM; i++)
    {
    }

    if (stat_rx_chn = dma_check_transfer_status(BOARD_HDMA, DBUS_UART_RX_DMA_CHN), stat_rx_chn & DMA_CHANNEL_STATUS_TC)
    {
    }
}
SDK_DECLARE_EXT_ISR_M(BOARD_HDMA_IRQ, HDma_Callback)

#elif RFL_CONFIG_CORE == RFL_CORE_RM_C_BORAD

#endif
