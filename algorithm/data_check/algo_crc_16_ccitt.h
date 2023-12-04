#ifndef _ALGO_CRC_16_CCITT_H__
#define _ALGO_CRC_16_CCITT_H__

#include "stddef.h"
#include "stdint.h"

/*
 * This mysterious table is just the CRC of each possible byte. It can be
 * computed using the standard bit-at-a-time methods. The polynomial can
 * be seen in entry 128, 0x8408. This corresponds to x^0 + x^5 + x^12.
 * Add the implicit x^16, and you have the standard CRC-CCITT.
 * https://github.com/torvalds/linux/blob/5bfc75d92efd494db37f5c4c173d3639d4772966/lib/crc-ccitt.c
 */

/**
 *	crc_ccitt - recompute the CRC (CRC-CCITT variant) for the data
 *	buffer
 *	@crc: previous CRC value
 *	@buffer: data pointer
 *	@len: number of bytes in the buffer
 */
uint16_t crc_ccitt(uint16_t crc, uint8_t const *buffer, size_t len);

#endif /* _ALGO_CRC_16_CCITT_H__ */
