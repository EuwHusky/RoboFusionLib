#include "bsp_ma600_middleware.h"

#if RFL_DEV_ENCODER_MA600

#define MA600_SPI HPM_SPI1 // SPI序号

// 定义片选引脚port和pin
#if (RFL_CONFIG_CORE == RFL_CORE_WPIE_HPM6750)
uint32_t group_of_cs_ports[RFL_DEV_ENCODER_MA600_USED_NUM] = {GPIO_DO_GPIOA};
uint8_t group_of_cs_pins[RFL_DEV_ENCODER_MA600_USED_NUM] = {6};

#elif (RFL_CONFIG_CORE == RFL_CORE_RM_C_BORAD)

GPIO_TypeDef *group_of_cs_ports[RFL_DEV_ENCODER_MA600_USED_NUM] = {CS_J2_GPIO_Port, CS_J3_GPIO_Port, CS_J4_GPIO_Port,
                                                                   CS_J6_GPIO_Port};
uint16_t group_of_cs_pins[RFL_DEV_ENCODER_MA600_USED_NUM] = {CS_J2_Pin, CS_J3_Pin, CS_J4_Pin, CS_J6_Pin};
#endif

// 初始化相关片选IO
void MA600_GPIO_init(void)
{
#if (RFL_CONFIG_CORE == RFL_CORE_WPIE_HPM6750)
    for (uint8_t i = 0; i < RFL_DEV_ENCODER_MA600_USED_NUM; i++)
        gpio_set_pin_output_with_initial(HPM_GPIO0, group_of_cs_ports[i], group_of_cs_pins[i], 1);

#elif (RFL_CONFIG_CORE == RFL_CORE_RM_C_BORAD)
    // 在 gpio.c 统一初始化

#endif
}

// 初始化SPI端口
void MA600_com_init(void)
{
#if (RFL_CONFIG_CORE == RFL_CORE_WPIE_HPM6750)
    board_init_spi_clock(MA600_SPI);         // 初始化时钟
    board_init_spi_pins(MA600_SPI);          // 初始化spi引脚
    spi_timing_config_t timing_config = {0}; // spi时钟配置
    spi_format_config_t format_config = {0}; // spi格式配置

    /* 设置主设备的SPI频率 */
    spi_master_get_default_timing_config(&timing_config); // 获取spi主机默认时序配置
    timing_config.master_config.clk_src_freq_in_hz = clock_get_frequency(clock_spi2);
    timing_config.master_config.sclk_freq_in_hz = 4000000; // 10000000,400000

    if (status_success != spi_master_timing_init(MA600_SPI, &timing_config))
    {
        printf("SPI master timing init failed\n"); // 时钟初始化失败输出报错
    }

    /* 为主设备设置 SPI 格式配置 */
    spi_master_get_default_format_config(&format_config);
    format_config.master_config.addr_len_in_bytes = 1;  // 地址长度
    format_config.common_config.data_len_in_bits = 8;   // 数据长度
    format_config.common_config.mode = spi_master_mode; // SPI工作模式
    spi_format_init(MA600_SPI, &format_config);

#elif (RFL_CONFIG_CORE == RFL_CORE_RM_C_BORAD)
    // 在 spi.c 统一初始化

#endif
}

void MA600_delay_ms(uint16_t ms)
{
#if (RFL_CONFIG_CORE == RFL_CORE_WPIE_HPM6750)

#elif (RFL_CONFIG_CORE == RFL_CORE_RM_C_BORAD)
    osDelay(ms);

#endif
}

#if (RFL_DEV_ENCODER_MA600_USED_NUM > 1)
// 拉低片选
void MA600_ACCEL_NS_L(uint8_t cs_num)
{
#if (RFL_CONFIG_CORE == RFL_CORE_WPIE_HPM6750)
    gpio_write_pin(HPM_GPIO0, group_of_cs_ports[cs_num], group_of_cs_pins[cs_num], 0);

#elif (RFL_CONFIG_CORE == RFL_CORE_RM_C_BORAD)
    HAL_GPIO_WritePin(group_of_cs_ports[cs_num], group_of_cs_pins[cs_num], GPIO_PIN_RESET);

#endif
}
// 拉高片选
void MA600_ACCEL_NS_H(uint8_t cs_num)
{
#if (RFL_CONFIG_CORE == RFL_CORE_WPIE_HPM6750)
    gpio_write_pin(HPM_GPIO0, group_of_cs_ports[cs_num], group_of_cs_pins[cs_num], 1);

#elif (RFL_CONFIG_CORE == RFL_CORE_RM_C_BORAD)
    HAL_GPIO_WritePin(group_of_cs_ports[cs_num], group_of_cs_pins[cs_num], GPIO_PIN_SET);

#endif
}
#else
// 拉低片选
void MA600_ACCEL_NS_L(void)
{
#if (RFL_CONFIG_CORE == RFL_CORE_WPIE_HPM6750)
    gpio_write_pin(HPM_GPIO0, group_of_cs_ports[0], group_of_cs_pins[0], 0);

#elif (RFL_CONFIG_CORE == RFL_CORE_RM_C_BORAD)
    HAL_GPIO_WritePin(group_of_cs_ports[0], group_of_cs_pins[0], GPIO_PIN_RESET);

#endif
}
// 拉高片选
void MA600_ACCEL_NS_H(void)
{
#if (RFL_CONFIG_CORE == RFL_CORE_WPIE_HPM6750)
    gpio_write_pin(HPM_GPIO0, group_of_cs_ports[0], group_of_cs_pins[0], 1);

#elif (RFL_CONFIG_CORE == RFL_CORE_RM_C_BORAD)
    HAL_GPIO_WritePin(group_of_cs_ports[0], group_of_cs_pins[0], GPIO_PIN_SET);

#endif
}
#endif

// 读写数据
bool MA600_read_write_byte(uint8_t *txdata, uint8_t *rxdata, uint8_t txdata_len, uint8_t rxdata_len)
{
#if (RFL_CONFIG_CORE == RFL_CORE_WPIE_HPM6750)
    hpm_stat_t stat;
    spi_control_config_t control_config = {0}; // spi控制配置
    spi_master_get_default_control_config(&control_config);
    control_config.common_config.trans_mode = spi_trans_write_read_together; // 传输模式
    stat = spi_transfer(MA600_SPI, &control_config, NULL, NULL, txdata, txdata_len, rxdata, rxdata_len);
    if (stat == status_success)
        return true;
    else
        return false;

#elif (RFL_CONFIG_CORE == RFL_CORE_RM_C_BORAD)
    uint32_t timeout = 10;
    HAL_SPI_TransmitReceive(&hspi2, txdata, rxdata, rxdata_len, timeout);

#endif

    return true;
}

#endif
