#ifndef __PID_CONTROLLER_H
#define __PID_CONTROLLER_H

#include <stdint.h>

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float previous_error;
    float output_min;
    float output_max;
    float integral_min;
    float integral_max;
    uint8_t initialized;
} PID_Controller;

void PID_Init(PID_Controller *pid,
              float kp, float ki, float kd,
              float output_min, float output_max,
              float integral_min, float integral_max);
void PID_Reset(PID_Controller *pid);
float PID_Update(PID_Controller *pid, float setpoint, float measurement, float dt);

#endif
