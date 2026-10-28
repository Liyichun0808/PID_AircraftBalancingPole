#include "main.h"
#include "gpio.h"
#include "tim.h"

#if !defined(PROJECT_MOTOR_PWM_TEST_BUILD)
#if defined(PROJECT_HARDWARE_I2C_BUILD)
#include "hardware_i2c.h"
#else
#include "software_i2c.h"
#endif
#include "mpu6050.h"
#include "balance_controller.h"
#endif

#define CONTROL_PERIOD_MS       10U
#define TIM2_PWM_PERIOD_COUNTS  100U

#if defined(PROJECT_DIRECT_RUN_CALIBRATED_BUILD)
#define CONTROL_CALIBRATED_START  1U
#else
#define CONTROL_CALIBRATED_START 0U
#endif

#if defined(PROJECT_INIT_FAIL_PWM_TEST_BUILD)
#define CONTROL_PROFILE_NAME     "init-fail-pwm-test"
#define CONTROL_OUTPUT_ENABLED   1U
#define CONTROL_AUTO_ARM         1U
#define CONTROL_DIRECT_RUN       1U
#elif defined(PROJECT_DIRECT_RUN_BUILD)
#define CONTROL_PROFILE_NAME     "project-direct-run"
#define CONTROL_OUTPUT_ENABLED   1U
#define CONTROL_AUTO_ARM         1U
#define CONTROL_DIRECT_RUN       1U
#elif defined(PROJECT_MOTOR_PWM_TEST_BUILD)
#define CONTROL_PROFILE_NAME     "motor-pwm-test"
#define CONTROL_OUTPUT_ENABLED   1U
#define CONTROL_AUTO_ARM         0U
#define CONTROL_DIRECT_RUN       0U
#elif defined(PROJECT_AUTOSTART_BUILD)
#define CONTROL_PROFILE_NAME     "project-autostart"
#define CONTROL_OUTPUT_ENABLED   1U
#define CONTROL_AUTO_ARM         1U
#define CONTROL_DIRECT_RUN       0U
#elif defined(PROJECT_RUNTIME_BUILD)
#define CONTROL_PROFILE_NAME     "project-runtime"
#define CONTROL_OUTPUT_ENABLED   1U
#define CONTROL_AUTO_ARM         0U
#define CONTROL_DIRECT_RUN       0U
#else
#define CONTROL_PROFILE_NAME     "hardware-test"
#define CONTROL_OUTPUT_ENABLED   0U
#define CONTROL_AUTO_ARM         0U
#define CONTROL_DIRECT_RUN       0U
#endif

#if !defined(PROJECT_MOTOR_PWM_TEST_BUILD)
/* Keep the controller configuration in one place so it can be tuned without
   changing the sensor driver or the hardware output code. */
static const BalanceController_Config controller_config =
{
  .target_angle_deg = 0.0f,
  .kp = 1.0f,
  .output_limit = 30.0f,
  /* Calibrate base commands and correction gains independently on hardware. */
  .base_pwm1 = 17.5f,
  .base_pwm2 = 20.0f,
  .correction_gain1 = 0.8765f,
  .correction_gain2 = 1.0f,
  .correction_sign = -1.0f,
  .arm_delay_ms = CONTROL_DIRECT_RUN ? 0U : 1000U,
  .arm_samples = CONTROL_DIRECT_RUN ? 1U : 50U,
  .max_angle_cdeg = 4500,
  .output_enabled = CONTROL_OUTPUT_ENABLED,
  .sensor_failure_limit = 3U
};

static uint8_t mpu6050_ok;
static uint32_t control_next_tick;
static MPU6050_RawData latest_raw;
static BalanceController balance_controller;
#endif

void SystemClock_Config(void);

#if !defined(PROJECT_MOTOR_PWM_TEST_BUILD)
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

static void Control_SetBaseMotorPwm(void)
{
  uint16_t pwm1;
  uint16_t pwm2;

  BalanceController_GetBasePwm(&controller_config, &pwm1, &pwm2);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm1);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm2);
}

static uint8_t Control_ReadMeasurement(float *angle_deg, float *gyro_dps)
{
  if (!MPU6050_ReadRaw(&latest_raw))
  {
    return 0U;
  }

  *angle_deg = MPU6050_UpdateAngleY(&latest_raw);
  *gyro_dps = MPU6050_GetGyroYFromRaw(&latest_raw);
  return 1U;
}
#endif

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM2_Init();

  /* The compare registers are explicitly zeroed before PWM is started. */
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0U);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0U);
  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

#if defined(PROJECT_INIT_FAIL_PWM_TEST_BUILD)
  /* Match MotorPwmTest's immediate 20% startup PWM before sensor checks. */
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 20U);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 20U);
#elif defined(PROJECT_DIRECT_RUN_BUILD) && !defined(PROJECT_DIRECT_RUN_CALIBRATED_BUILD)
  /* Apply the configured base command immediately at power-up. */
  Control_SetBaseMotorPwm();
#endif

#if defined(PROJECT_MOTOR_PWM_TEST_BUILD)
  /* Bench profile: fixed PWM output without MPU6050 or PID startup gates. */
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 20U);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 20U);
  while (1)
  {
  }
#else
  BalanceController_Init(&balance_controller,
                         &controller_config,
                         HAL_GetTick());

#if defined(PROJECT_HARDWARE_I2C_BUILD)
  if (!HW_I2C_Init() || !HW_I2C_BusReady())
#else
  SW_I2C_Init();
  if (!SW_I2C_BusReady())
#endif
  {
    mpu6050_ok = 0U;
  }
  else
  {
    mpu6050_ok = MPU6050_Init();
  }
  if (!mpu6050_ok && CONTROL_DIRECT_RUN)
  {
#if defined(PROJECT_INIT_FAIL_PWM_TEST_BUILD)
    /* Diagnostic profile: keep a visible PWM output after MPU/I2C init fails. */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 50U);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 50U);
#else
    /* Remove the startup base command when feedback cannot be established. */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0U);
#endif
  }
  if (mpu6050_ok && (!CONTROL_DIRECT_RUN || CONTROL_CALIBRATED_START))
  {
    /* Keep the board still while the gyro bias is measured. */
    if (!MPU6050_Calibrate(100U))
    {
      mpu6050_ok = 0U;
    }
    else if (CONTROL_CALIBRATED_START)
    {
      Control_SetBaseMotorPwm();
    }
  }

#if CONTROL_AUTO_ARM
  if (mpu6050_ok)
  {
    BalanceController_RequestArm(&balance_controller);
  }
#endif

  control_next_tick = HAL_GetTick() + CONTROL_PERIOD_MS;

  while (1)
  {
    uint32_t now = HAL_GetTick();

    /* Execute the control slot on fixed deadlines, independent of I2C duration. */
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
  }
#endif
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
