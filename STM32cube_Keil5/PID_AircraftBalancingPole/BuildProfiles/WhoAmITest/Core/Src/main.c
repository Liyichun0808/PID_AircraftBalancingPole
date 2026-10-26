#include "main.h"
#include "gpio.h"
#include "software_i2c.h"

#define MPU_ADDR_68_WRITE  0xD0U
#define MPU_ADDR_69_WRITE  0xD2U
#define MPU_WHO_AM_I       0x75U

#define LED_PORT           GPIOC
#define LED_PIN            GPIO_PIN_13

typedef enum
{
  MPU_ID_I2C_ERROR = 0U,
  MPU_ID_UNKNOWN,
  MPU_ID_6050,
  MPU_ID_6500,
  MPU_ID_9250,
  MPU_ID_9255
} MpuIdResult;

void SystemClock_Config(void);

static void Led_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
  gpio.Pin = LED_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PORT, &gpio);
}

static MpuIdResult Read_MpuId(void)
{
  const uint8_t addresses[] = {MPU_ADDR_68_WRITE, MPU_ADDR_69_WRITE};
  uint8_t id = 0U;
  uint8_t read_succeeded = 0U;

  if (!SW_I2C_BusReady())
  {
    return MPU_ID_I2C_ERROR;
  }

  for (uint32_t i = 0U; i < sizeof(addresses); i++)
  {
    if (SW_I2C_Read(addresses[i], MPU_WHO_AM_I, &id, 1U))
    {
      read_succeeded = 1U;
      switch (id)
      {
        case 0x68U: return MPU_ID_6050;
        case 0x70U: return MPU_ID_6500;
        case 0x71U: return MPU_ID_9250;
        case 0x73U: return MPU_ID_9255;
        default:    break;
      }
    }
  }

  return read_succeeded ? MPU_ID_UNKNOWN : MPU_ID_I2C_ERROR;
}

static void Led_ShowResult(MpuIdResult result)
{
  uint8_t flashes;

  switch (result)
  {
    case MPU_ID_6050: flashes = 1U; break;
    case MPU_ID_6500: flashes = 2U; break;
    case MPU_ID_9250: flashes = 3U; break;
    case MPU_ID_9255: flashes = 4U; break;
    case MPU_ID_UNKNOWN: flashes = 5U; break;
    default: flashes = 8U; break;
  }

  for (uint8_t i = 0U; i < flashes; i++)
  {
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
    HAL_Delay(150U);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
    HAL_Delay(250U);
  }
  HAL_Delay(1500U);
}

int main(void)
{
  MpuIdResult result;

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  Led_Init();
  SW_I2C_Init();

  result = Read_MpuId();
  while (1)
  {
    Led_ShowResult(result);
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
