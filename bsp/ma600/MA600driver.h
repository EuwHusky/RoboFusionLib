#ifndef MA600_DRIVER_H
#define MA600_DRIVER_H

#include "MA600Middleware.h"

extern bool MA600_init(void); // MA600初始化

#if (NUM_OF_MA600 > 1)
extern uint16_t MA600_read_with_check(bool *error, uint8_t num);                   // 带校验的读取
extern uint16_t MA600_read(uint8_t num);                                           // 直接读取
extern uint8_t readMagAlphaRegister(uint8_t address, uint8_t num);                 // 读取寄存器
extern uint8_t writeMagAlphaRegister(uint8_t address, uint8_t value, uint8_t num); // 写入寄存器
#else
extern uint16_t MA600_read_with_check(bool *error);                   // 带校验的读取
extern uint16_t MA600_read(void);                                     // 直接读取
extern uint8_t readMagAlphaRegister(uint8_t address);                 // 读取寄存器
extern uint8_t writeMagAlphaRegister(uint8_t address, uint8_t value); // 写入寄存器
#endif

#endif
