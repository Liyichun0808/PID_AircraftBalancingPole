// MyCode/software_i2c.h
#ifndef __SOFTWARE_I2C_H
#define __SOFTWARE_I2C_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

// ============ I2C ���Ŷ��� ============
#define I2C_SCL_PIN    GPIO_PIN_6
#define I2C_SCL_PORT   GPIOB
#define I2C_SDA_PIN    GPIO_PIN_7
#define I2C_SDA_PORT   GPIOB

// ============ I2C ������ ============
#define SCL_HIGH()   HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET)
#define SCL_LOW()    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_RESET)
#define SDA_HIGH()   HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_SET)
#define SDA_LOW()    HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_RESET)
#define SDA_READ()   HAL_GPIO_ReadPin(I2C_SDA_PORT, I2C_SDA_PIN)

// ============ �������� ============
void SW_I2C_Init(void);
uint8_t SW_I2C_BusReady(void);
uint8_t SW_I2C_Write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
uint8_t SW_I2C_Read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
void SW_I2C_Scan(void);

#endif
