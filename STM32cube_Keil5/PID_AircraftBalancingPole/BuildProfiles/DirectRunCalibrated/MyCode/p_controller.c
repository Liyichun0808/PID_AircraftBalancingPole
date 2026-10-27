#include "p_controller.h"

static float P_Clamp(float value, float minimum, float maximum)
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

void P_Init(P_Controller *controller,
            float kp,
            float output_min,
            float output_max)
{
    controller->kp = kp;
    controller->output_min = output_min;
    controller->output_max = output_max;
}

float P_Update(const P_Controller *controller,
               float setpoint,
               float measurement)
{
    float error = setpoint - measurement;
    float output = controller->kp * error;

    return P_Clamp(output, controller->output_min, controller->output_max);
}
