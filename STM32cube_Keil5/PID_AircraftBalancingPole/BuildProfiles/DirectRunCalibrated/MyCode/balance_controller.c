#include "balance_controller.h"

static float Balance_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static uint8_t Balance_FloatIsValid(float value)
{
    /* NaN is the only IEEE-754 value that is not equal to itself. The broad
       bound also rejects infinities before any float-to-integer conversion. */
    return (value == value && value <= 10000.0f && value >= -10000.0f) ?
           1U : 0U;
}

static uint16_t Balance_PwmCount(float value)
{
    value = Balance_Clamp(value, 0.0f, 99.0f);
    return (uint16_t)(value + 0.5f);
}

static void Balance_ClearOutput(BalanceController *controller)
{
    controller->pwm1 = 0U;
    controller->pwm2 = 0U;
    controller->correction_milli = 0;
}

static void Balance_Disarm(BalanceController *controller, uint8_t latch_fault)
{
    controller->armed = 0U;
    controller->arm_requested = 0U;
    controller->stable_samples = 0U;
    controller->correction_milli = 0;
    Balance_ClearOutput(controller);
    if (latch_fault != 0U)
    {
        controller->faulted = 1U;
    }
}

void BalanceController_Init(BalanceController *controller,
                            const BalanceController_Config *config,
                            uint32_t now_ms)
{
    controller->config = *config;
    P_Init(&controller->p_controller,
           config->kp,
           -config->output_limit,
           config->output_limit);
    controller->angle_cdeg = 0;
    controller->gyro_mdps = 0;
    controller->sensor_failure_count = 0U;
    controller->stable_samples = 0U;
    controller->sensor_healthy = 0U;
    controller->arm_requested = 0U;
    controller->armed = 0U;
    controller->faulted = 0U;
    controller->boot_tick = now_ms;
    Balance_ClearOutput(controller);
}

void BalanceController_RequestArm(BalanceController *controller)
{
    if (controller->faulted == 0U)
    {
        controller->arm_requested = 1U;
    }
}

void BalanceController_RequestDisarm(BalanceController *controller)
{
    Balance_Disarm(controller, 0U);
}

void BalanceController_Update(BalanceController *controller,
                              uint8_t sample_valid,
                              float angle_deg,
                              float gyro_dps,
                              uint32_t now_ms)
{
    float correction;
    float angle_cdeg_float;

    if (sample_valid == 0U ||
        Balance_FloatIsValid(angle_deg) == 0U ||
        Balance_FloatIsValid(gyro_dps) == 0U)
    {
        controller->sensor_healthy = 0U;
        if (controller->sensor_failure_count <
            controller->config.sensor_failure_limit)
        {
            controller->sensor_failure_count++;
        }
        if (controller->sensor_failure_count >=
            controller->config.sensor_failure_limit)
        {
            Balance_Disarm(controller, 1U);
        }
        return;
    }

    controller->sensor_failure_count = 0U;
    controller->sensor_healthy = 1U;
    angle_cdeg_float = angle_deg * 100.0f;
    controller->angle_cdeg = (int32_t)angle_cdeg_float;
    controller->gyro_mdps = (int32_t)(gyro_dps * 1000.0f);

    if (controller->angle_cdeg > controller->config.max_angle_cdeg ||
        controller->angle_cdeg < -controller->config.max_angle_cdeg)
    {
        Balance_Disarm(controller, 1U);
        return;
    }

    if (controller->stable_samples < controller->config.arm_samples)
    {
        controller->stable_samples++;
    }

    if (controller->config.output_enabled != 0U &&
        controller->faulted == 0U &&
        controller->arm_requested != 0U &&
        controller->stable_samples >= controller->config.arm_samples &&
        (uint32_t)(now_ms - controller->boot_tick) >=
        controller->config.arm_delay_ms)
    {
        controller->armed = 1U;
    }

    if (controller->armed == 0U)
    {
        Balance_ClearOutput(controller);
        return;
    }

    correction = P_Update(&controller->p_controller,
                          controller->config.target_angle_deg,
                          angle_deg);
    correction *= controller->config.correction_sign;
    controller->correction_milli = (int32_t)(correction * 1000.0f);
    controller->pwm1 = Balance_PwmCount(controller->config.base_pwm + correction);
    controller->pwm2 = Balance_PwmCount(controller->config.base_pwm - correction);
}

void BalanceController_GetPwm(const BalanceController *controller,
                              uint16_t *pwm1,
                              uint16_t *pwm2)
{
    *pwm1 = controller->pwm1;
    *pwm2 = controller->pwm2;
}

uint8_t BalanceController_IsArmed(const BalanceController *controller)
{
    return controller->armed;
}

uint8_t BalanceController_IsArmRequested(const BalanceController *controller)
{
    return controller->arm_requested;
}

uint8_t BalanceController_IsFaulted(const BalanceController *controller)
{
    return controller->faulted;
}

uint8_t BalanceController_IsHealthy(const BalanceController *controller)
{
    return controller->sensor_healthy;
}

uint8_t BalanceController_GetSensorFailureCount(const BalanceController *controller)
{
    return controller->sensor_failure_count;
}

int32_t BalanceController_GetAngleCdeg(const BalanceController *controller)
{
    return controller->angle_cdeg;
}

int32_t BalanceController_GetGyroMdps(const BalanceController *controller)
{
    return controller->gyro_mdps;
}

int32_t BalanceController_GetCorrectionMilli(const BalanceController *controller)
{
    return controller->correction_milli;
}

uint16_t BalanceController_GetStableSamples(const BalanceController *controller)
{
    return controller->stable_samples;
}
