#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"
#include "software_i2c.h"
#include "mpu6050.h"
#include "pid_controller.h"
#include <stdio.h>

#define CONTROL_PERIOD_MS       10U
#define SENSOR_FAILURE_LIMIT    3U
#define TIM2_PWM_PERIOD_COUNTS  100U

/* Initial gains are expressed as PWM counts per degree and per degree/second.
   They are deliberately conservative starting values and must be tuned on the
   assembled mechanism. */
#define PID_TARGET_ANGLE_DEG    0.0f
#define PID_KP                  1.0f
#define PID_KI                  0.05f
#define PID_KD                  0.08f
#define PID_OUTPUT_LIMIT        30.0f
#define PID_INTEGRAL_LIMIT      100.0f
#define CONTROL_MAX_ANGLE_CDEG  4500
#define CONTROL_ARM_SAMPLES     50U
#define CONTROL_ARM_DELAY_MS    1000U

/* Keep actuation disabled until motor polarity, sensor axis, and gains are
   confirmed on the real assembly. Set to 1 only after those checks. */
#define CONTROL_OUTPUT_ENABLE   0U
#define MOTOR_BASE_PWM          20.0f
#define MIX_CORRECTION_SIGN     1.0f

static uint8_t mpu6050_ok;
static uint32_t control_next_tick;
static uint32_t report_tick;
static MPU6050_RawData latest_raw;
static int32_t latest_angle_x_cdeg;
static int32_t latest_gyro_x_mdps;
static int32_t latest_pid_correction_milli;
static uint8_t latest_raw_valid;
static uint8_t sensor_healthy;
static uint8_t sensor_failure_count;
static uint16_t stable_sample_count;
static uint8_t controller_armed;
static uint32_t boot_tick;
static PID_Controller attitude_pid;

void SystemClock_Config(void);

static float Control_ClampPwm(float value)
{
  if (value < 0.0f)
  {
    return 0.0f;
  }
  if (value > (float)(TIM2_PWM_PERIOD_COUNTS - 1U))
  {
    return (float)(TIM2_PWM_PERIOD_COUNTS - 1U);
  }
  return value;
}

static void Control_SetMotorPwm(float pwm1, float pwm2)
{
  if (CONTROL_OUTPUT_ENABLE == 0U || controller_armed == 0U)
  {
    pwm1 = 0.0f;
    pwm2 = 0.0f;
  }

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,
                        (uint32_t)(Control_ClampPwm(pwm1) + 0.5f));
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,
                        (uint32_t)(Control_ClampPwm(pwm2) + 0.5f));
}

static uint8_t Control_UpdateMeasurement(void)
{
  if (!MPU6050_ReadRaw(&latest_raw))
  {
    latest_raw_valid = 0U;
    return 0U;
  }

  latest_angle_x_cdeg =
      (int32_t)(MPU6050_UpdateAngleX(&latest_raw) * 100.0f);
  latest_gyro_x_mdps =
      (int32_t)(MPU6050_GetGyroXFromRaw(&latest_raw) * 1000.0f);
  latest_raw_valid = 1U;
  return 1U;
}

static void Control_UpdatePid(void)
{
  float angle_deg = (float)latest_angle_x_cdeg / 100.0f;
  float gyro_x_dps = (float)latest_gyro_x_mdps / 1000.0f;
  float correction = PID_UpdateWithRate(&attitude_pid,
                                        PID_TARGET_ANGLE_DEG,
                                        angle_deg,
                                        gyro_x_dps,
                                        (float)CONTROL_PERIOD_MS / 1000.0f);

  latest_pid_correction_milli = (int32_t)(correction * 1000.0f);
  correction *= MIX_CORRECTION_SIGN;
  Control_SetMotorPwm(MOTOR_BASE_PWM + correction,
                      MOTOR_BASE_PWM - correction);
}

static void Control_Disarm(void)
{
  controller_armed = 0U;
  stable_sample_count = 0U;
  latest_pid_correction_milli = 0;
  PID_Reset(&attitude_pid);
  Control_SetMotorPwm(0.0f, 0.0f);
}

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

  PID_Init(&attitude_pid,
           PID_KP, PID_KI, PID_KD,
           -PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT,
           -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);

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
  sensor_healthy = mpu6050_ok;
  sensor_failure_count = 0U;
  stable_sample_count = 0U;
  controller_armed = 0U;
  boot_tick = HAL_GetTick();
  control_next_tick = HAL_GetTick() + CONTROL_PERIOD_MS;
  report_tick = HAL_GetTick();

  while (1)
  {
    uint32_t now = HAL_GetTick();

    /* Run the measurement slot on a fixed deadline so loop execution time
       does not accumulate into the 10 ms control period. */
    if (sensor_healthy &&
        (int32_t)(now - control_next_tick) >= 0)
    {
      do
      {
        control_next_tick += CONTROL_PERIOD_MS;
      }
      while ((int32_t)(now - control_next_tick) >= 0);

      if (Control_UpdateMeasurement())
      {
        sensor_failure_count = 0U;
        if (latest_angle_x_cdeg > CONTROL_MAX_ANGLE_CDEG ||
            latest_angle_x_cdeg < -CONTROL_MAX_ANGLE_CDEG)
        {
          Control_Disarm();
        }
        else
        {
          if (stable_sample_count < CONTROL_ARM_SAMPLES)
          {
            stable_sample_count++;
          }
          if (CONTROL_OUTPUT_ENABLE != 0U &&
              stable_sample_count >= CONTROL_ARM_SAMPLES &&
              (uint32_t)(now - boot_tick) >= CONTROL_ARM_DELAY_MS)
          {
            controller_armed = 1U;
          }
          if (controller_armed != 0U)
          {
            Control_UpdatePid();
          }
          else
          {
            latest_pid_correction_milli = 0;
            PID_Reset(&attitude_pid);
            Control_SetMotorPwm(0.0f, 0.0f);
          }
        }
      }
      else
      {
        Control_Disarm();
        if (sensor_failure_count < SENSOR_FAILURE_LIMIT)
        {
          sensor_failure_count++;
          if (sensor_failure_count >= SENSOR_FAILURE_LIMIT)
          {
            sensor_healthy = 0U;
            Control_Disarm();
          }
        }
      }
    }

    if ((uint32_t)(now - report_tick) >= 500U)
    {
      report_tick = now;
      if (!sensor_healthy)
      {
        printf("controller disarmed, sensor unavailable\r\n");
      }
      else if (latest_raw_valid)
      {
        printf("raw ax=%d ay=%d az=%d gx=%d gy=%d gz=%d\r\n",
               latest_raw.accel_x, latest_raw.accel_y, latest_raw.accel_z,
               latest_raw.gyro_x, latest_raw.gyro_y, latest_raw.gyro_z);
        printf("angle_x_cdeg=%ld\r\n", (long)latest_angle_x_cdeg);
        printf("gyro_x_mdps=%ld pid_correction_milli=%ld output=%u\r\n",
               (long)latest_gyro_x_mdps,
               (long)latest_pid_correction_milli,
               (unsigned)CONTROL_OUTPUT_ENABLE);
        printf("armed=%u stable_samples=%u\r\n",
               (unsigned)controller_armed,
               (unsigned)stable_sample_count);
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
