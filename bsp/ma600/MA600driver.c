#include "MA600driver.h"

bool MA600_init(void)
{
    MA600_GPIO_init();
    MA600_com_init();
}

#if (NUM_OF_MA600 > 1)
// 直接读取
uint16_t MA600_read(uint8_t num)
{
    uint32_t timeout = 10;
    uint8_t txData[2];
    uint8_t rxData[2];
    txData[1] = 0;
    txData[0] = 0;
    uint16_t angleSensor;
    MA600_ACCEL_NS_L(num);
    MA600_read_write_byte((uint8_t *)txData, (uint8_t *)rxData, 2, 2);
    MA600_ACCEL_NS_H(num);
    angleSensor = rxData[0] << 8 | rxData[1];
    return angleSensor;
}

// 带校验的读取
uint16_t MA600_read_with_check(bool *error, uint8_t num)
{
    uint32_t timeout = 10;
    uint8_t highStateCount = 0;
    uint8_t parity;
    uint8_t txData[3];
    uint8_t rxData[3];
    txData[2] = 0;
    txData[1] = 0;
    txData[0] = 0;
    uint16_t angleSensor;
    MA600_ACCEL_NS_L(num);
    MA600_read_write_byte((uint8_t *)txData, (uint8_t *)rxData, 3, 3);
    MA600_ACCEL_NS_H(num);
    angleSensor = rxData[0] << 8 | rxData[1];
    parity = ((rxData[2] & 0x80) >> 7);
    // Count the number of 1 in the angle binary value
    for (int i = 0; i < 16; ++i)
    {
        if ((angleSensor & (1 << i)) != 0)
        {
            highStateCount++;
        }
    }
    // check if parity bit is correct
    if ((highStateCount % 2) == 0) // number of bits set to 1 in the angle is even
    {
        if (parity == 0)
        {
            *error = false;
        }
        else
        {
            *error = true;
        }
    }
    else // number of bits set to 1 in the angle is odd
    {
        if (parity == 1)
        {
            *error = false;
        }
        else
        {
            *error = true;
        }
    }
    return angleSensor;
}

// 读取寄存器
uint8_t readMagAlphaRegister(uint8_t address, uint8_t num)
{
    uint32_t timeout = 10;
    uint32_t delay = 1; // ms
    uint8_t txData1[2];
    uint8_t rxData1[2];
    uint8_t txData2[2];
    uint8_t rxData2[2];
    txData1[0] = (0x2 << 5) | (0x1F & address);
    txData1[1] = 0x00;
    txData2[0] = 0x00;
    txData2[1] = 0x00;
    uint8_t registerReadbackValue;
    MA600_ACCEL_NS_L(num);
    MA600_read_write_byte((uint8_t *)txData1, (uint8_t *)rxData1, 2, 2);
    MA600_ACCEL_NS_H(num);
    HAL_Delay(delay);
    MA600_ACCEL_NS_L(num);
    MA600_read_write_byte((uint8_t *)txData2, (uint8_t *)rxData2, 2, 2);
    MA600_ACCEL_NS_H(num);
    registerReadbackValue = rxData2[0];
    return registerReadbackValue;
}

// 写入寄存器
uint8_t writeMagAlphaRegister(uint8_t address, uint8_t value, uint8_t num)
{
    hpm_stat_t stat;
    spi_control_config_t control_config = {0}; // spi控制配置
    spi_master_get_default_control_config(&control_config);
    control_config.common_config.trans_mode = spi_trans_write_read_together; // 传输模式

    uint32_t timeout = 10;
    uint32_t delay = 20; // ms
    uint8_t txData1[2];
    uint8_t rxData1[2];
    uint8_t txData2[2];
    uint8_t rxData2[2];
    txData1[0] = (0x4 << 5) | (0x1F & address);
    txData1[1] = value;
    txData2[0] = 0x00;
    txData2[1] = 0x00;
    uint8_t registerReadbackValue;
    MA600_ACCEL_NS_L(num);
    MA600_read_write_byte((uint8_t *)txData1, (uint8_t *)rxData1, 2, 2);
    MA600_ACCEL_NS_H(num);
    HAL_Delay(delay);
    MA600_ACCEL_NS_L(num);
    MA600_read_write_byte((uint8_t *)txData2, (uint8_t *)rxData2, 2, 2);
    MA600_ACCEL_NS_H(num);
    registerReadbackValue = rxData2[0];
    return registerReadbackValue;
}
#else
// 直接读取
uint16_t MA600_read(void)
{
    uint32_t timeout = 10;
    uint8_t txData[2];
    uint8_t rxData[2];
    txData[1] = 0;
    txData[0] = 0;
    uint16_t angleSensor;
    MA600_ACCEL_NS_L();
    MA600_read_write_byte((uint8_t *)txData, (uint8_t *)rxData, 2, 2);
    MA600_ACCEL_NS_H();
    angleSensor = rxData[0] << 8 | rxData[1];
    return angleSensor;
}

// 带校验的读取
uint16_t MA600_read_with_check(bool *error)
{
    uint32_t timeout = 10;
    uint8_t highStateCount = 0;
    uint8_t parity;
    uint8_t txData[3];
    uint8_t rxData[3];
    txData[2] = 0;
    txData[1] = 0;
    txData[0] = 0;
    uint16_t angleSensor;
    MA600_ACCEL_NS_L();
    MA600_read_write_byte((uint8_t *)txData, (uint8_t *)rxData, 3, 3);
    MA600_ACCEL_NS_H();
    angleSensor = rxData[0] << 8 | rxData[1];
    parity = ((rxData[2] & 0x80) >> 7);
    // Count the number of 1 in the angle binary value
    for (int i = 0; i < 16; ++i)
    {
        if ((angleSensor & (1 << i)) != 0)
        {
            highStateCount++;
        }
    }
    // check if parity bit is correct
    if ((highStateCount % 2) == 0) // number of bits set to 1 in the angle is even
    {
        if (parity == 0)
        {
            *error = false;
        }
        else
        {
            *error = true;
        }
    }
    else // number of bits set to 1 in the angle is odd
    {
        if (parity == 1)
        {
            *error = false;
        }
        else
        {
            *error = true;
        }
    }
    return angleSensor;
}

// 读取寄存器
uint8_t readMagAlphaRegister(uint8_t address)
{
    uint32_t timeout = 10;
    uint32_t delay = 1; // ms
    uint8_t txData1[2];
    uint8_t rxData1[2];
    uint8_t txData2[2];
    uint8_t rxData2[2];
    txData1[0] = (0x2 << 5) | (0x1F & address);
    txData1[1] = 0x00;
    txData2[0] = 0x00;
    txData2[1] = 0x00;
    uint8_t registerReadbackValue;
    MA600_ACCEL_NS_L();
    MA600_read_write_byte((uint8_t *)txData1, (uint8_t *)rxData1, 2, 2);
    MA600_ACCEL_NS_H();
    // HAL_Delay(delay);
    MA600_ACCEL_NS_L();
    MA600_read_write_byte((uint8_t *)txData2, (uint8_t *)rxData2, 2, 2);
    MA600_ACCEL_NS_H();
    registerReadbackValue = rxData2[0];
    return registerReadbackValue;
}

// 写入寄存器
uint8_t writeMagAlphaRegister(uint8_t address, uint8_t value)
{
    hpm_stat_t stat;
    spi_control_config_t control_config = {0}; // spi控制配置
    spi_master_get_default_control_config(&control_config);
    control_config.common_config.trans_mode = spi_trans_write_read_together; // 传输模式

    uint32_t timeout = 10;
    uint32_t delay = 20; // ms
    uint8_t txData1[2];
    uint8_t rxData1[2];
    uint8_t txData2[2];
    uint8_t rxData2[2];
    txData1[0] = (0x4 << 5) | (0x1F & address);
    txData1[1] = value;
    txData2[0] = 0x00;
    txData2[1] = 0x00;
    uint8_t registerReadbackValue;
    MA600_ACCEL_NS_L();
    MA600_read_write_byte((uint8_t *)txData1, (uint8_t *)rxData1, 2, 2);
    MA600_ACCEL_NS_H();
    // HAL_Delay(delay);
    MA600_ACCEL_NS_L();
    MA600_read_write_byte((uint8_t *)txData2, (uint8_t *)rxData2, 2, 2);
    MA600_ACCEL_NS_H();
    registerReadbackValue = rxData2[0];
    return registerReadbackValue;
}
#endif
