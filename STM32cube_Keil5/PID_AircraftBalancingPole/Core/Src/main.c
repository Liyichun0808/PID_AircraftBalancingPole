#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"
#include "software_i2c.h"
#include "mpu6050.h"
#include <stdio.h>

#define PWM_TEST_COMPARE 20U

static uint8_t mpu6050_ok;
static uint32_t sensor_tick;

void SystemClock_Config(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();

  /* PA1/TIM2_CH2 and PA2/TIM2_CH3: fixed 20% PWM for step 1. */
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM_TEST_COMPARE);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM_TEST_COMPARE);

  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  printf("step3 pwm started\r\n");
  SW_I2C_Init();
  mpu6050_ok = MPU6050_Init();
  printf("step3 mpu6050=%s\r\n", mpu6050_ok ? "OK" : "FAIL");
  sensor_tick = HAL_GetTick();

  while (1)
  {
    if (HAL_GetTick() - sensor_tick >= 500U)
    {
      sensor_tick = HAL_GetTick();
      if (mpu6050_ok)
      {
        MPU6050_RawData raw;
        if (MPU6050_ReadRaw(&raw))
        {
          printf("raw ax=%d ay=%d az=%d gx=%d gy=%d gz=%d\r\n",
                 raw.accel_x, raw.accel_y, raw.accel_z,
                 raw.gyro_x, raw.gyro_y, raw.gyro_z);
        }
        else
        {
          printf("raw read failed\r\n");
        }
      }
      else
      {
        printf("step3 alive, sensor unavailable\r\n");
      }
    }
  }
}

int fputc(int ch, FILE *file)
{
  (void)file;
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1U, 100U);
  return ch;
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};

  osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  osc.HSIState = RCC_HSI_ON;
  osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  osc.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK)
  {
    Error_Handler();
  }

  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV1;
  clk.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
