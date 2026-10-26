#ifndef __HARDWARE_I2C_H
#define __HARDWARE_I2C_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

extern I2C_HandleTypeDef hi2c1;

uint8_t HW_I2C_Init(void);
uint8_t HW_I2C_BusReady(void);
uint8_t HW_I2C_Write(uint8_t dev_addr, uint8_t reg_addr,
                     uint8_t *data, uint16_t len);
uint8_t HW_I2C_Read(uint8_t dev_addr, uint8_t reg_addr,
                    uint8_t *data, uint16_t len);

#endif
