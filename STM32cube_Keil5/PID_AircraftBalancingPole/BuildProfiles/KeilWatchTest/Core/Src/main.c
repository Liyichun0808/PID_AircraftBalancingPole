#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "software_i2c.h"
#include "mpu6050.h"
#include "balance_controller.h"
#include "keil_watch_test.h"

#define CONTROL_PERIOD_MS 10U
#define SENSOR_STARTUP_MS  200U
#define SENSOR_RETRY_MS    500U
#define LED_PORT           GPIOC
#define LED_PIN            GPIO_PIN_13

/* Match the DirectRunCalibrated P controller for command comparison.
   output_enabled only arms the SOFTWARE controller here. No controller
   command is ever written to the hardware PWM registers in this file. */
static const BalanceController_Config controller_config =
{
    .target_angle_deg = 0.0f,
    .kp = 1.0f,
    .output_limit = 30.0f,
    .base_pwm1 = 20.0f,
    .base_pwm2 = 20.0f,
    .correction_gain1 = 1.0f,
    .correction_gain2 = 1.0f,
    .correction_sign = -1.0f,
    .arm_delay_ms = 0U,
    .arm_samples = 1U,
    .max_angle_cdeg = 4500,
    .output_enabled = 1U,
    .sensor_failure_limit = 3U
};

volatile KeilWatchData g_keil_watch;
volatile KeilWatchData g_keil_samples[3];
volatile uint32_t g_keil_capture_request;
volatile uint32_t g_keil_capture_result;
volatile int16_t sensor_accel_raw[3];
volatile int16_t sensor_gyro_raw[3];

static BalanceController balance_controller;
static MPU6050_RawData latest_raw;
static uint32_t previous_sample_tick;
static uint8_t have_previous_sample;

void SystemClock_Config(void);

static void Watch_ReadBusLines(void)
{
    g_keil_watch.scl_high =
        HAL_GPIO_ReadPin(I2C_SCL_PORT, I2C_SCL_PIN) == GPIO_PIN_SET ? 1U : 0U;
    g_keil_watch.sda_high =
        HAL_GPIO_ReadPin(I2C_SDA_PORT, I2C_SDA_PIN) == GPIO_PIN_SET ? 1U : 0U;
}

static void Watch_InitPins(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* Mirror SensorAxisTest's PC13 initialization before touching the MPU. */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
    gpio.Pin = LED_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &gpio);

    /* Keep PA1/PA2 low while sensor startup runs before TIM2 initialization. */
    HAL_GPIO_WritePin(GPIOA, PWM1_Pin | PWM2_Pin, GPIO_PIN_RESET);
    gpio.Pin = PWM1_Pin | PWM2_Pin;
    HAL_GPIO_Init(GPIOA, &gpio);
}

static void Watch_KeepMotorsOff(void)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0U);
    g_keil_watch.pwm1_actual = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2);
    g_keil_watch.pwm2_actual = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_3);
    g_keil_watch.motors_enabled = 0U;
}

static void Watch_PublishController(void)
{
    g_keil_watch.armed = BalanceController_IsArmed(&balance_controller);
    g_keil_watch.faulted = BalanceController_IsFaulted(&balance_controller);
    g_keil_watch.sensor_healthy = BalanceController_IsHealthy(&balance_controller);
    g_keil_watch.sensor_failure_count =
        BalanceController_GetSensorFailureCount(&balance_controller);
    g_keil_watch.angle_cdeg = BalanceController_GetAngleCdeg(&balance_controller);
    g_keil_watch.gyro_mdps = BalanceController_GetGyroMdps(&balance_controller);
    g_keil_watch.correction_milli =
        BalanceController_GetCorrectionMilli(&balance_controller);
    g_keil_watch.correction_pwm = (float)g_keil_watch.correction_milli / 1000.0f;
    g_keil_watch.pwm1_calc = balance_controller.pwm1;
    g_keil_watch.pwm2_calc = balance_controller.pwm2;

    if (g_keil_watch.faulted != 0U)
    {
        g_keil_watch.stage = KEIL_STAGE_CONTROLLER_FAULT;
        if (g_keil_watch.fault_reason == 0U)
        {
            if (g_keil_watch.sensor_healthy == 0U)
            {
                g_keil_watch.fault_reason = 2U;
            }
            else if (g_keil_watch.angle_cdeg > controller_config.max_angle_cdeg ||
                     g_keil_watch.angle_cdeg < -controller_config.max_angle_cdeg)
            {
                g_keil_watch.fault_reason = 1U;
            }
            else
            {
                g_keil_watch.fault_reason = 3U;
            }
        }
    }
}

static void Watch_Capture(void)
{
    uint32_t slot = g_keil_capture_request;

    if (slot == 0U)
    {
        return;
    }
    if (slot > 3U)
    {
        g_keil_capture_result = 2U;
    }
    else if (g_keil_watch.sample_valid == 0U || g_keil_watch.sensor_healthy == 0U ||
             g_keil_watch.armed == 0U ||
             g_keil_watch.faulted != 0U)
    {
        g_keil_capture_result = 3U;
    }
    else
    {
        g_keil_samples[slot - 1U] = g_keil_watch;
        g_keil_samples[slot - 1U].snapshot_valid = 1U;
        g_keil_capture_result = 1U;
    }
    /* Publish acknowledgement only after the complete snapshot is stored. */
    g_keil_capture_request = 0U;
}

static void Watch_Sample(uint32_t now)
{
    uint32_t read_start = HAL_GetTick();
    float angle_deg = 0.0f;
    float gyro_dps = 0.0f;
    uint8_t valid = MPU6050_ReadRaw(&latest_raw);

    g_keil_watch.read_duration_ms = HAL_GetTick() - read_start;
    g_keil_watch.sample_interval_ms = have_previous_sample ?
                                         now - previous_sample_tick : 0U;
    previous_sample_tick = now;
    have_previous_sample = 1U;
    g_keil_watch.sample_valid = valid;

    if (valid != 0U)
    {
        angle_deg = MPU6050_UpdateAngleY(&latest_raw);
        gyro_dps = MPU6050_GetGyroYFromRaw(&latest_raw);
        g_keil_watch.angle_deg = angle_deg;
        g_keil_watch.gyro_dps = gyro_dps;
        g_keil_watch.error_deg = controller_config.target_angle_deg - angle_deg;
        g_keil_watch.accel_x_raw = latest_raw.accel_x;
        g_keil_watch.accel_y_raw = latest_raw.accel_y;
        g_keil_watch.accel_z_raw = latest_raw.accel_z;
        g_keil_watch.gyro_x_raw = latest_raw.gyro_x;
        g_keil_watch.gyro_y_raw = latest_raw.gyro_y;
        g_keil_watch.gyro_z_raw = latest_raw.gyro_z;
        sensor_accel_raw[0] = latest_raw.accel_x;
        sensor_accel_raw[1] = latest_raw.accel_y;
        sensor_accel_raw[2] = latest_raw.accel_z;
        sensor_gyro_raw[0] = latest_raw.gyro_x;
        sensor_gyro_raw[1] = latest_raw.gyro_y;
        sensor_gyro_raw[2] = latest_raw.gyro_z;
    }
    else
    {
        /* Preserve the last measurement for inspection; sample_valid marks it stale. */
        g_keil_watch.read_failure_total++;
    }

    BalanceController_Update(&balance_controller, valid, angle_deg, gyro_dps, now);
    Watch_PublishController();
    Watch_KeepMotorsOff();
    Watch_ReadBusLines();
    g_keil_watch.sample_count++;
    Watch_Capture();
}

static void Watch_TrySensorInit(void)
{
    g_keil_watch.init_attempts++;
    g_keil_watch.sample_valid = 0U;
    g_keil_watch.mpu_ok = 0U;
    g_keil_watch.calibrated = 0U;
    g_keil_watch.stage = KEIL_STAGE_I2C;
    /* Same bus setup, readiness gate, device identification and register
       writes as SensorAxisTest. Never bypass a failed bus check. */
    SW_I2C_Init();
    Watch_ReadBusLines();
    if (!SW_I2C_BusReady())
    {
        g_keil_watch.stage = KEIL_STAGE_BUS_ERROR;
    }
    else
    {
        g_keil_watch.stage = KEIL_STAGE_SENSOR_INIT;
        g_keil_watch.mpu_ok = MPU6050_Init();
        if (g_keil_watch.mpu_ok == 0U)
        {
            g_keil_watch.stage = KEIL_STAGE_SENSOR_ERROR;
        }
        else
        {
            g_keil_watch.stage = KEIL_STAGE_CALIBRATION;
            g_keil_watch.calibrated = MPU6050_Calibrate(100U);
            if (g_keil_watch.calibrated == 0U)
            {
                g_keil_watch.stage = KEIL_STAGE_CALIBRATION_ERROR;
            }
            else
            {
                BalanceController_Init(&balance_controller, &controller_config,
                                       HAL_GetTick());
                BalanceController_RequestArm(&balance_controller);
                have_previous_sample = 0U;
                g_keil_watch.stage = KEIL_STAGE_RUNNING;
            }
        }
    }
    Watch_ReadBusLines();
    if (g_keil_watch.calibrated == 0U)
    {
        g_keil_watch.last_init_error = g_keil_watch.stage;
    }
    HAL_GPIO_WritePin(LED_PORT, LED_PIN,
                     g_keil_watch.calibrated != 0U ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

int main(void)
{
    uint32_t next_tick;
    uint32_t next_init_tick;

    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    Watch_InitPins();
    BalanceController_Init(&balance_controller, &controller_config, HAL_GetTick());

    /* SensorAxisTest initializes the sensor before any PWM peripheral.
       Add a short power-up settling time, then follow that same order. */
    g_keil_watch.stage = KEIL_STAGE_I2C;
    HAL_Delay(SENSOR_STARTUP_MS);
    Watch_TrySensorInit();

    MX_TIM2_Init();
    Watch_KeepMotorsOff();
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }

    next_tick = HAL_GetTick() + CONTROL_PERIOD_MS;
    next_init_tick = HAL_GetTick() + SENSOR_RETRY_MS;
    while (1)
    {
        uint32_t now = HAL_GetTick();

        if ((int32_t)(now - next_tick) >= 0)
        {
            do
            {
                next_tick += CONTROL_PERIOD_MS;
            }
            while ((int32_t)(now - next_tick) >= 0);

            g_keil_watch.tick_ms = now;
            Watch_KeepMotorsOff();
            if (g_keil_watch.mpu_ok != 0U && g_keil_watch.calibrated != 0U)
            {
                Watch_Sample(now);
            }
            else
            {
                Watch_ReadBusLines();
                Watch_Capture();
                /* Retry initialization only. Runtime safety faults stay latched
                   and cannot trigger automatic recalibration or re-arming. */
                if ((int32_t)(now - next_init_tick) >= 0)
                {
                    Watch_TrySensorInit();
                    Watch_KeepMotorsOff();
                    next_init_tick = HAL_GetTick() + SENSOR_RETRY_MS;
                    next_tick = HAL_GetTick() + CONTROL_PERIOD_MS;
                }
            }
        }
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
    if (htim2.Instance == TIM2)
    {
        Watch_KeepMotorsOff();
    }
    g_keil_watch.stage = KEIL_STAGE_HARDWARE_ERROR;
    __disable_irq();
    while (1)
    {
    }
}
