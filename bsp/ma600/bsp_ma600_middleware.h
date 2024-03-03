#ifndef _BSP_MA600_MIDDLEWARE_H__
#define _BSP_MA600_MIDDLEWARE_H__

#include "stdbool.h"
#include "stdint.h"

#include "rfl_config.h"

#if RFL_DEV_ENCODER_MA600

#if (RFL_CONFIG_CORE == RFL_CORE_WPIE_HPM6750)
#include "board.h"
#include "hpm_gpio_drv.h"
#include "hpm_gpiom_drv.h"
#include "hpm_spi_drv.h"

#elif (RFL_CONFIG_CORE == RFL_CORE_RM_C_BORAD)
#include "cmsis_os.h"
#include "main.h"
extern SPI_HandleTypeDef hspi2;
#endif

#define NUM_OF_MA600 2 // 使用的MA600个数

extern void MA600_GPIO_init(void);
extern void MA600_com_init(void);

extern void MA600_delay_ms(uint16_t ms);

#if (NUM_OF_MA600 > 1)
extern void MA600_ACCEL_NS_L(uint8_t cs_num);
extern void MA600_ACCEL_NS_H(uint8_t cs_num);
#else
extern void MA600_ACCEL_NS_L(void);
extern void MA600_ACCEL_NS_H(void);
#endif

extern bool MA600_read_write_byte(uint8_t *txdata, uint8_t *rxdata, uint8_t txdata_len, uint8_t rxdata_len);

#endif

#endif /* _BSP_MA600_MIDDLEWARE_H__ */
