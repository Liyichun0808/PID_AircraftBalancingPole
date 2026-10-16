#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"
#include "software_i2c.h"
#include "mpu6050.h"
#include "balance_controller.h"
#include "control_console.h"
#include <stdio.h>

#define CONTROL_PERIOD_MS       10U
#define TIM2_PWM_PERIOD_COUNTS  100U

#if defined(PROJECT_RUNTIME_BUILD)
#define CONTROL_PROFILE_NAME     "project-runtime"
#define CONTROL_OUTPUT_ENABLED   1U
#else
#define CONTROL_PROFILE_NAME     "hardware-test"
#define CONTROL_OUTPUT_ENABLED   0U
#endif

/* Keep the controller configuration in one place so it can be tuned without
   changing the sensor driver or the hardware output code. */
static const BalanceController_Config controller_config =
{
  .target_angle_deg = 0.0f,
  .kp = 1.0f,
  .ki = 0.05f,
  .kd = 0.08f,
  .output_limit = 30.0f,
  .integral_limit = 100.0f,
  .base_pwm = 20.0f,
  .correction_sign = 1.0f,
  .period_ms = CONTROL_PERIOD_MS,
  .arm_delay_ms = 1000U,
  .arm_samples = 50U,
  .max_angle_cdeg = 4500,
  .output_enabled = CONTROL_OUTPUT_ENABLED,
  .sensor_failure_limit = 3U
};

static uint8_t mpu6050_ok;
static uint8_t latest_raw_valid;
static uint32_t control_next_tick;
static uint32_t report_tick;
static MPU6050_RawData latest_raw;
static BalanceController balance_controller;

void SystemClock_Config(void);

static void Control_SetMotorPwm(void)
{
  uint16_t pwm1;
  uint16_t pwm2;

  BalanceController_GetPwm(&balance_controller, &pwm1, &pwm2);
  if (pwm1 >= TIM2_PWM_PERIOD_COUNTS)
  {
    pwm1 = TIM2_PWM_PERIOD_COUNTS - 1U;
  }
  if (pwm2 >= TIM2_PWM_PERIOD_COUNTS)
  {
    pwm2 = TIM2_PWM_PERIOD_COUNTS - 1U;
  }

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm1);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm2);
}

static uint8_t Control_ReadMeasurement(float *angle_deg, float *gyro_dps)
{
  if (!MPU6050_ReadRaw(&latest_raw))
  {
    latest_raw_valid = 0U;
    return 0U;
  }

  *angle_deg = MPU6050_UpdateAngleX(&latest_raw);
  *gyro_dps = MPU6050_GetGyroXFromRaw(&latest_raw);
  latest_raw_valid = 1U;
  return 1U;
}

static void Control_PrintStatus(void)
{
  uint16_t pwm1;
  uint16_t pwm2;

  BalanceController_GetPwm(&balance_controller, &pwm1, &pwm2);
  printf("healthy=%u arm_req=%u armed=%u fault=%u stable=%u fail=%u output=%u\r\n",
         (unsigned)BalanceController_IsHealthy(&balance_controller),
         (unsigned)BalanceController_IsArmRequested(&balance_controller),
         (unsigned)BalanceController_IsArmed(&balance_controller),
         (unsigned)BalanceController_IsFaulted(&balance_controller),
         (unsigned)BalanceController_GetStableSamples(&balance_controller),
         (unsigned)BalanceController_GetSensorFailureCount(&balance_controller),
         (unsigned)controller_config.output_enabled);
  printf("angle_x_cdeg=%ld gyro_x_mdps=%ld pid_correction_milli=%ld integral_milli=%ld pwm1=%u pwm2=%u\r\n",
         (long)BalanceController_GetAngleCdeg(&balance_controller),
         (long)BalanceController_GetGyroMdps(&balance_controller),
         (long)BalanceController_GetCorrectionMilli(&balance_controller),
         (long)BalanceController_GetIntegralMilli(&balance_controller),
         (unsigned)pwm1,
         (unsigned)pwm2);
}

static void Control_ProcessConsole(void)
{
  ControlCommand command = ControlConsole_Poll();

  switch (command)
  {
    case CONTROL_COMMAND_ARM:
      BalanceController_RequestArm(&balance_controller);
      printf("arm requested\r\n");
      break;
    case CONTROL_COMMAND_DISARM:
      BalanceController_RequestDisarm(&balance_controller);
      Control_SetMotorPwm();
      printf("disarmed\r\n");
      break;
    case CONTROL_COMMAND_STATUS:
      Control_PrintStatus();
      break;
    default:
      break;
  }
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();

  /* The compare registers are explicitly zeroed before PWM is started. */
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0U);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0U);
  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  printf("control profile=%s output_enabled=%u\r\n",
         CONTROL_PROFILE_NAME,
         (unsigned)controller_config.output_enabled);
  ControlConsole_Init(&huart1);
  BalanceController_Init(&balance_controller,
                         &controller_config,
                         HAL_GetTick());

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
    /* Keep the board still while the gyro bias is measured. */
    if (!MPU6050_Calibrate(100U))
    {
      mpu6050_ok = 0U;
      printf("mpu6050 calibration failed\r\n");
    }
  }

  control_next_tick = HAL_GetTick() + CONTROL_PERIOD_MS;
  report_tick = HAL_GetTick();

  while (1)
  {
    uint32_t now = HAL_GetTick();

    Control_ProcessConsole();

    /* Execute the control slot on fixed deadlines, independent of UART
       traffic and the variable duration of the I2C transaction. */
    if (mpu6050_ok && (int32_t)(now - control_next_tick) >= 0)
    {
      float angle_deg;
      float gyro_dps;

      do
      {
        control_next_tick += CONTROL_PERIOD_MS;
      }
      while ((int32_t)(now - control_next_tick) >= 0);

      if (Control_ReadMeasurement(&angle_deg, &gyro_dps))
      {
        BalanceController_Update(&balance_controller,
                                 1U,
                                 angle_deg,
                                 gyro_dps,
                                 now);
      }
      else
      {
        BalanceController_Update(&balance_controller,
                                 0U,
                                 0.0f,
                                 0.0f,
                                 now);
      }
      Control_SetMotorPwm();
    }

    if ((uint32_t)(now - report_tick) >= 500U)
    {
      report_tick = now;
      if (!mpu6050_ok || !BalanceController_IsHealthy(&balance_controller))
      {
        printf("controller disarmed, sensor unavailable\r\n");
      }
      else if (latest_raw_valid)
      {
        printf("raw ax=%d ay=%d az=%d gx=%d gy=%d gz=%d\r\n",
               latest_raw.accel_x, latest_raw.accel_y, latest_raw.accel_z,
               latest_raw.gyro_x, latest_raw.gyro_y, latest_raw.gyro_z);
        Control_PrintStatus();
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
