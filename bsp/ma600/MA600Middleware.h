#ifndef MA600_MIDDLEWARE_H
#define MA600_MIDDLEWARE_H

#include "board.h"
#include "hpm_gpio_drv.h"
#include "hpm_gpiom_drv.h"
#include "hpm_spi_drv.h"
#include "rfl_config.h"

#define NUM_OF_MA600 1 // 使用的MA600个数

extern void MA600_GPIO_init(void);
extern void MA600_com_init(void);

#if (NUM_OF_MA600 > 1)
extern void MA600_ACCEL_NS_L(uint8_t cs_num);
extern void MA600_ACCEL_NS_H(uint8_t cs_num);
#else
extern void MA600_ACCEL_NS_L(void);
extern void MA600_ACCEL_NS_H(void);
#endif

extern bool MA600_read_write_byte(uint8_t *txdata, uint8_t *rxdata, uint8_t txdata_len, uint8_t rxdata_len);

#endif