#ifndef __BSP_BMI088MIDDLEWARE_H_
#define __BSP_BMI088MIDDLEWARE_H_

#include "stdint.h"

#define BMI088_USE_SPI
// #define BMI088_USE_IIC

// // ========================== 硬件 SPI 驱动 ==========================
// #define BMI088_SPI_SPEED (100 * 1000)       // 硬件 SPI 速率
// #define BMI088_SPI (SPI_2)                  // 硬件 SPI 号
// #define BMI088_SPC_PIN (SPI2_MAP0_SCK_B13)  // 硬件 SPI SCK 引脚
// #define BMI088_SDI_PIN (SPI2_MAP0_MOSI_B15) // 硬件 SPI MOSI 引脚
// #define BMI088_SDO_PIN (SPI2_MAP0_MISO_B14) // 硬件 SPI MISO 引脚
// // ========================== 硬件 SPI 驱动 ==========================
// #define BMI088_ACCLE_CS_PIN (D10) // 加速度计 CS 片选引脚
// #define BMI088_GYRO_CS_PIN (D11)  // 陀螺仪 CS 片选引脚
// #define BMI088_ACCLE_CS(x) ((x) ? (gpio_high(BMI088_ACCLE_CS_PIN)) : (gpio_low(BMI088_ACCLE_CS_PIN)))
// #define BMI088_GYRO_CS(x) ((x) ? (gpio_high(BMI088_GYRO_CS_PIN)) : (gpio_low(BMI088_GYRO_CS_PIN)))

extern void BMI088_GPIO_init(void);
extern void BMI088_com_init(void);
extern void BMI088_delay_ms(uint16_t ms);
extern void BMI088_delay_us(uint16_t us);

#if defined(BMI088_USE_SPI)
extern void BMI088_ACCEL_NS_L(void);
extern void BMI088_ACCEL_NS_H(void);

extern void BMI088_GYRO_NS_L(void);
extern void BMI088_GYRO_NS_H(void);

extern uint8_t BMI088_read_write_byte(uint8_t reg);

#elif defined(BMI088_USE_IIC)

#endif

#endif /* __BSP_BMI088MIDDLEWARE_H_ */
