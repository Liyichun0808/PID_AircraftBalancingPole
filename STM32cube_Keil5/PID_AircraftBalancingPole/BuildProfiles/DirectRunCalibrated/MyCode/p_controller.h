#ifndef P_CONTROLLER_H
#define P_CONTROLLER_H

typedef struct
{
    float kp;
    float output_min;
    float output_max;
} P_Controller;

void P_Init(P_Controller *controller,
            float kp,
            float output_min,
            float output_max);
float P_Update(const P_Controller *controller,
               float setpoint,
               float measurement);

#endif
