#include "hardware_i2c.h"
#include <stddef.h>

I2C_HandleTypeDef hi2c1;
static uint8_t i2c1_initialized;

uint8_t HW_I2C_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_I2C1_CLK_ENABLE();

  gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  gpio.Mode = GPIO_MODE_AF_OD;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio);

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000U;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0U;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0U;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  i2c1_initialized = (HAL_I2C_Init(&hi2c1) == HAL_OK) ? 1U : 0U;
  return i2c1_initialized;
}

uint8_t HW_I2C_BusReady(void)
{
  if (!i2c1_initialized)
  {
    return 0U;
  }

  return (HAL_I2C_IsDeviceReady(&hi2c1, 0xD0U, 3U, 100U) == HAL_OK) ? 1U : 0U;
}

uint8_t HW_I2C_Write(uint8_t dev_addr, uint8_t reg_addr,
                     uint8_t *data, uint16_t len)
{
  if (!i2c1_initialized || (data == NULL) || (len == 0U))
  {
    return 0U;
  }

  return (HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(dev_addr & 0xFEU),
                            reg_addr, I2C_MEMADD_SIZE_8BIT,
                            data, len, 100U) == HAL_OK) ? 1U : 0U;
}

uint8_t HW_I2C_Read(uint8_t dev_addr, uint8_t reg_addr,
                    uint8_t *data, uint16_t len)
{
  if (!i2c1_initialized || (data == NULL) || (len == 0U))
  {
    return 0U;
  }

  return (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(dev_addr & 0xFEU),
                           reg_addr, I2C_MEMADD_SIZE_8BIT,
                           data, len, 100U) == HAL_OK) ? 1U : 0U;
}
