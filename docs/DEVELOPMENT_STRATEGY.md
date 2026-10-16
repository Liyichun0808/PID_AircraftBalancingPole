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

Until hardware or a working sensor model is available, validation is limited
to compilation, link/HEX generation, deterministic arithmetic checks, and
inspection of the control-loop bounds. Runtime sensor values and PID gains
must be confirmed on the actual assembly.
