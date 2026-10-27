#ifndef __BALANCE_CONTROLLER_H
#define __BALANCE_CONTROLLER_H

#include <stdint.h>
#include "p_controller.h"

typedef struct
{
    float target_angle_deg;
    float kp;
    float output_limit;
    float base_pwm;
    float correction_sign;
    uint32_t arm_delay_ms;
    uint16_t arm_samples;
    int32_t max_angle_cdeg;
    uint8_t output_enabled;
    uint8_t sensor_failure_limit;
} BalanceController_Config;

typedef struct
{
    P_Controller p_controller;
    BalanceController_Config config;
    int32_t angle_cdeg;
    int32_t gyro_mdps;
    int32_t correction_milli;
    uint16_t pwm1;
    uint16_t pwm2;
    uint16_t stable_samples;
    uint8_t sensor_failure_count;
    uint8_t sensor_healthy;
    uint8_t arm_requested;
    uint8_t armed;
    uint8_t faulted;
    uint32_t boot_tick;
} BalanceController;

void BalanceController_Init(BalanceController *controller,
                            const BalanceController_Config *config,
                            uint32_t now_ms);
void BalanceController_RequestArm(BalanceController *controller);
void BalanceController_RequestDisarm(BalanceController *controller);
void BalanceController_Update(BalanceController *controller,
                              uint8_t sample_valid,
                              float angle_deg,
                              float gyro_dps,
                              uint32_t now_ms);
void BalanceController_GetPwm(const BalanceController *controller,
                              uint16_t *pwm1,
                              uint16_t *pwm2);
uint8_t BalanceController_IsArmed(const BalanceController *controller);
uint8_t BalanceController_IsArmRequested(const BalanceController *controller);
uint8_t BalanceController_IsFaulted(const BalanceController *controller);
uint8_t BalanceController_IsHealthy(const BalanceController *controller);
uint8_t BalanceController_GetSensorFailureCount(const BalanceController *controller);
int32_t BalanceController_GetAngleCdeg(const BalanceController *controller);
int32_t BalanceController_GetGyroMdps(const BalanceController *controller);
int32_t BalanceController_GetCorrectionMilli(const BalanceController *controller);
uint16_t BalanceController_GetStableSamples(const BalanceController *controller);

#endif
