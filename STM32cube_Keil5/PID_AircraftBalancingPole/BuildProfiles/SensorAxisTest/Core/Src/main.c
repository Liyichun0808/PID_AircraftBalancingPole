#include "main.h"
#include "gpio.h"
#include "software_i2c.h"
#include "mpu6050.h"
#include <stdint.h>

#define LED_PORT              GPIOC
#define LED_PIN               GPIO_PIN_13
#define AXIS_CAPTURE_SAMPLES  30U
#define GYRO_CAPTURE_MS       2000U
#define GYRO_MOTION_THRESHOLD 300L

void SystemClock_Config(void);

volatile int16_t sensor_accel_raw[3];
volatile int16_t sensor_gyro_raw[3];

static void AxisTest_Led(uint8_t on)
{
  HAL_GPIO_WritePin(LED_PORT, LED_PIN,
                    on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void AxisTest_Flash(uint8_t count)
{
  uint8_t i;

  for (i = 0U; i < count; i++)
  {
    AxisTest_Led(1U);
    HAL_Delay(150U);
    AxisTest_Led(0U);
    HAL_Delay(300U);
  }
}

static void AxisTest_ShowResult(uint8_t mode, uint8_t axis, int32_t sign)
{
  /* Marker: one short flash means accelerometer; two mean gyroscope. */
  AxisTest_Flash(mode);
  HAL_Delay(500U);

  if (axis >= 1U && axis <= 3U)
  {
    AxisTest_Flash(axis);
    AxisTest_Led(1U);
    HAL_Delay(sign >= 0 ? 400U : 1400U);
    AxisTest_Led(0U);
  }
  else
  {
    /* Four flashes mean no dominant gravity axis or no detected rotation. */
    AxisTest_Flash(4U);
  }

  HAL_Delay(2200U);
}

static uint8_t AxisTest_Read(MPU6050_RawData *raw)
{
  if (!MPU6050_ReadRaw(raw))
  {
    return 0U;
  }

  sensor_accel_raw[0] = raw->accel_x;
  sensor_accel_raw[1] = raw->accel_y;
  sensor_accel_raw[2] = raw->accel_z;
  sensor_gyro_raw[0] = raw->gyro_x;
  sensor_gyro_raw[1] = raw->gyro_y;
  sensor_gyro_raw[2] = raw->gyro_z;
  return 1U;
}

static uint8_t AxisTest_FindAccelAxis(const MPU6050_RawData *raw)
{
  int32_t values[3];
  uint32_t magnitudes[3];
  uint8_t best = 0U;
  uint8_t i;

  values[0] = raw->accel_x;
  values[1] = raw->accel_y;
  values[2] = raw->accel_z;
  for (i = 0U; i < 3U; i++)
  {
    magnitudes[i] = (values[i] < 0) ? (uint32_t)(-values[i]) :
                                      (uint32_t)values[i];
    if (magnitudes[i] > magnitudes[best])
    {
      best = i;
    }
  }
  if (magnitudes[best] < 5000U)
  {
    return 0U;
  }
  return (uint8_t)(best + 1U);
}

static int32_t AxisTest_GetAccelSign(const MPU6050_RawData *raw, uint8_t axis)
{
  if (axis == 1U) return raw->accel_x;
  if (axis == 2U) return raw->accel_y;
  return raw->accel_z;
}

static uint8_t AxisTest_CaptureGyro(uint8_t *axis, int32_t *sign)
{
  MPU6050_RawData raw;
  int32_t peak = 0;
  uint8_t peak_axis = 0U;
  uint32_t start = HAL_GetTick();

  /* LED stays on for the 2-second window: rotate one axis during this time. */
  AxisTest_Led(1U);
  while ((uint32_t)(HAL_GetTick() - start) < GYRO_CAPTURE_MS)
  {
    int32_t values[3];
    uint8_t i;

    if (!AxisTest_Read(&raw))
    {
      AxisTest_Led(0U);
      return 0U;
    }

    values[0] = raw.gyro_x;
    values[1] = raw.gyro_y;
    values[2] = raw.gyro_z;
    for (i = 0U; i < 3U; i++)
    {
      int32_t magnitude = (values[i] < 0) ? -values[i] : values[i];
      if (magnitude > peak)
      {
        peak = magnitude;
        peak_axis = (uint8_t)(i + 1U);
        *sign = values[i];
      }
    }
    HAL_Delay(10U);
  }
  AxisTest_Led(0U);

  if (peak < GYRO_MOTION_THRESHOLD)
  {
    *axis = 0U;
  }
  else
  {
    *axis = peak_axis;
  }
  return 1U;
}

int main(void)
{
  GPIO_InitTypeDef gpio = {0};
  MPU6050_RawData raw;
  int32_t accel_sum[3];
  int32_t gyro_sign = 0;
  uint8_t accel_axis;
  uint8_t gyro_axis;
  uint8_t i;

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

  __HAL_RCC_GPIOC_CLK_ENABLE();
  gpio.Pin = LED_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PORT, &gpio);
  AxisTest_Led(0U);

  SW_I2C_Init();
  if (!SW_I2C_BusReady() || !MPU6050_Init())
  {
    while (1)
    {
      AxisTest_Flash(8U);
      HAL_Delay(1500U);
    }
  }

  while (1)
  {
    accel_sum[0] = 0U;
    accel_sum[1] = 0U;
    accel_sum[2] = 0U;

    for (i = 0U; i < AXIS_CAPTURE_SAMPLES; i++)
    {
      if (!AxisTest_Read(&raw))
      {
        AxisTest_Flash(8U);
        HAL_Delay(1500U);
        break;
      }
      accel_sum[0] += raw.accel_x;
      accel_sum[1] += raw.accel_y;
      accel_sum[2] += raw.accel_z;
      HAL_Delay(10U);
    }
    if (i < AXIS_CAPTURE_SAMPLES)
    {
      continue;
    }

    raw.accel_x = (int16_t)(accel_sum[0] / (int32_t)AXIS_CAPTURE_SAMPLES);
    raw.accel_y = (int16_t)(accel_sum[1] / (int32_t)AXIS_CAPTURE_SAMPLES);
    raw.accel_z = (int16_t)(accel_sum[2] / (int32_t)AXIS_CAPTURE_SAMPLES);
    accel_axis = AxisTest_FindAccelAxis(&raw);
    AxisTest_ShowResult(1U, accel_axis,
                        AxisTest_GetAccelSign(&raw, accel_axis));

    if (!AxisTest_CaptureGyro(&gyro_axis, &gyro_sign))
    {
      AxisTest_Flash(8U);
      HAL_Delay(1500U);
      continue;
    }
    AxisTest_ShowResult(2U, gyro_axis, gyro_sign);
  }
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
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();

  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV2;
  clk.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
