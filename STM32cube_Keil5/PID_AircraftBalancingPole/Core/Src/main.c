#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"
#include <stdio.h>

#define PWM_TEST_COMPARE 20U

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

  printf("step2 start\r\n");

  while (1)
  {
    printf("step2 alive\r\n");
    HAL_Delay(1000U);
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
