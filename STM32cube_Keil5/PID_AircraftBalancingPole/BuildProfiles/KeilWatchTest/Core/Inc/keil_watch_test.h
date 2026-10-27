#ifndef KEIL_WATCH_TEST_H
#define KEIL_WATCH_TEST_H

#include <stdint.h>

typedef enum
{
    KEIL_STAGE_BOOT = 0,
    KEIL_STAGE_I2C = 1,
    KEIL_STAGE_SENSOR_INIT = 2,
    KEIL_STAGE_CALIBRATION = 3,
    KEIL_STAGE_RUNNING = 4,
    KEIL_STAGE_CONTROLLER_FAULT = 5,
    KEIL_STAGE_BUS_ERROR = 100,
    KEIL_STAGE_SENSOR_ERROR = 101,
    KEIL_STAGE_CALIBRATION_ERROR = 102,
    KEIL_STAGE_HARDWARE_ERROR = 103
} KeilWatchStage;

typedef struct
{
    uint32_t stage;
    uint32_t tick_ms;
    uint32_t sample_count;
    uint32_t read_failure_total;
    uint32_t sample_interval_ms;
    uint32_t read_duration_ms;
    uint32_t snapshot_valid;
    uint32_t sample_valid;
    uint32_t mpu_ok;
    uint32_t calibrated;
    uint32_t armed;
    uint32_t faulted;
    /* Latched reason: 0 = none, 1 = angle, 2 = sensor, 3 = other. */
    uint32_t fault_reason;
    uint32_t sensor_healthy;
    uint32_t sensor_failure_count;
    uint32_t motors_enabled;
    int32_t angle_cdeg;
    int32_t gyro_mdps;
    int32_t correction_milli;
    float angle_deg;
    float gyro_dps;
    float error_deg;
    float correction_pwm;
    uint32_t pwm1_calc;
    uint32_t pwm2_calc;
    /* Actual TIM2 CCR2/CCR3: always zero in this observation-only build. */
    uint32_t pwm1_actual;
    uint32_t pwm2_actual;
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;
} KeilWatchData;

extern volatile KeilWatchData g_keil_watch;
extern volatile KeilWatchData g_keil_samples[3];
/* Write 1/2/3 to capture upright / toward PWM1 / toward PWM2. */
extern volatile uint32_t g_keil_capture_request;
/* 0 = idle, 1 = saved, 2 = bad slot, 3 = not ready; request clears on handling. */
extern volatile uint32_t g_keil_capture_result;

#endif
