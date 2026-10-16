#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"
#include "software_i2c.h"
#include "mpu6050.h"
#include <stdio.h>

static uint8_t mpu6050_ok;
static uint32_t sample_tick;
static uint32_t report_tick;
static MPU6050_RawData latest_raw;
static int32_t latest_angle_x_cdeg;
static uint8_t latest_raw_valid;

void SystemClock_Config(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();

  /* Keep both motor outputs disabled until the closed-loop controller is armed. */
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0U);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0U);

  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  printf("control outputs disabled at boot\r\n");
  SW_I2C_Init();
  if (!SW_I2C_BusReady())
  {
    printf("i2c bus not ready\r\n");
    mpu6050_ok = 0U;
  }
  else
  {
    printf("i2c bus ready\r\n");
    mpu6050_ok = MPU6050_Init();
  }
  printf("mpu6050=%s\r\n", mpu6050_ok ? "OK" : "FAIL");
  if (mpu6050_ok)
  {
    /* The board must remain motionless during this startup calibration. */
    MPU6050_Calibrate(100U);
  }
  sample_tick = HAL_GetTick();
  report_tick = sample_tick;

  while (1)
  {
    uint32_t now = HAL_GetTick();

    if (mpu6050_ok && (uint32_t)(now - sample_tick) >= 10U)
    {
      sample_tick = now;
      if (MPU6050_ReadRaw(&latest_raw))
      {
        latest_angle_x_cdeg =
            (int32_t)(MPU6050_UpdateAngleX(&latest_raw) * 100.0f);
        latest_raw_valid = 1U;
      }
      else
      {
        latest_raw_valid = 0U;
      }
    }

    if ((uint32_t)(now - report_tick) >= 500U)
    {
      report_tick = now;
      if (!mpu6050_ok)
      {
        printf("controller disarmed, sensor unavailable\r\n");
      }
      else if (latest_raw_valid)
      {
        printf("raw ax=%d ay=%d az=%d gx=%d gy=%d gz=%d\r\n",
               latest_raw.accel_x, latest_raw.accel_y, latest_raw.accel_z,
               latest_raw.gyro_x, latest_raw.gyro_y, latest_raw.gyro_z);
        printf("angle_x_cdeg=%ld\r\n", (long)latest_angle_x_cdeg);
      }
      else
      {
        printf("raw read failed\r\n");
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

  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState = RCC_HSE_ON;
  osc.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  osc.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK)
  {
    Error_Handler();
  }

  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV2;
  clk.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK)
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
