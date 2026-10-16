# Development Strategy

This project is developed in stages so each layer can be checked independently.

1. Fixed PWM output on PA1/PA2.
2. USART1 heartbeat and diagnostics.
3. Software I2C bus check and MPU6050 register initialization.
4. Raw sensor reads and filtered attitude telemetry.
5. Standalone PID math with output limits and integral anti-windup.
6. A fixed-rate control loop and motor PWM mixing.
7. Calibration and gain tuning on hardware.

The board clock is the 8 MHz external crystal multiplied to a 72 MHz system
clock. APB1 is 36 MHz and TIM2 receives the doubled 72 MHz timer clock; a
prescaler of 719 and period of 99 give a 1 kHz PWM base. Motor outputs remain
at zero until sensor initialization, calibration, axis sign, and the control
loop have been verified.

The measurement slot now runs from a fixed 10 ms deadline. A successful raw
sample updates the filtered attitude estimate; three consecutive I2C read
failures disarm the outputs. This health gate is retained by the PID and
two-motor mixer stage below.

The PID stage uses the fused angle for proportional/integral action and the
calibrated gyro rate for derivative damping, avoiding a noisy finite difference
of the angle estimate. The mixer uses equal and opposite correction around a
configurable base PWM. `CONTROL_OUTPUT_ENABLE` remains 0 until the assembled
mechanism's axis direction, motor polarity, base throttle, and gains are
verified. Even when enabled, actuation requires 50 valid 10 ms samples and a
one-second startup delay; an attitude excursion beyond +/-45 degrees or any
sensor fault disarms both outputs immediately.

Until hardware or a working sensor model is available, validation is limited
to compilation, link/HEX generation, deterministic arithmetic checks, and
inspection of the control-loop bounds. Runtime sensor values and PID gains
must be confirmed on the actual assembly.
