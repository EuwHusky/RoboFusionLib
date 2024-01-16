#ifndef _BSP_USART_H__
#define _BSP_USART_H__

#include "rfl_config.h"

#if RFL_CONFIG_CORE == RFL_CORE_RM_C_BORAD

#include "stdint.h"

extern void usart1_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void usart1_tx_dma_enable(uint8_t *data, uint16_t len);

extern void usart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void usart6_tx_dma_enable(uint8_t *data, uint16_t len);

#endif /* RFL_CONFIG_CORE == RFL_CORE_RM_C_BORAD */

#endif /* _BSP_USART_H__ */
