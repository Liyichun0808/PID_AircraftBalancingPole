#include "pid_controller.h"

static float PID_Clamp(float value, float minimum, float maximum)
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

void PID_Init(PID_Controller *pid,
              float kp, float ki, float kd,
              float output_min, float output_max,
              float integral_min, float integral_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->integral_min = integral_min;
    pid->integral_max = integral_max;
    PID_Reset(pid);
}

void PID_Reset(PID_Controller *pid)
{
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->initialized = 0U;
}

float PID_Update(PID_Controller *pid, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;
    float derivative = 0.0f;
    float integral_candidate;
    float unsaturated;
    float output;

    if (dt <= 0.0f)
    {
        return 0.0f;
    }

    if (pid->initialized != 0U)
    {
        derivative = (error - pid->previous_error) / dt;
    }

    integral_candidate = PID_Clamp(pid->integral + error * dt,
                                   pid->integral_min,
                                   pid->integral_max);
    unsaturated = pid->kp * error +
                  pid->ki * integral_candidate +
                  pid->kd * derivative;
    output = PID_Clamp(unsaturated, pid->output_min, pid->output_max);

    /* Do not integrate farther into an active output limit. */
    if ((unsaturated == output) ||
        ((output >= pid->output_max) && (error < 0.0f)) ||
        ((output <= pid->output_min) && (error > 0.0f)))
    {
        pid->integral = integral_candidate;
    }

    pid->previous_error = error;
    pid->initialized = 1U;
    return output;
}
