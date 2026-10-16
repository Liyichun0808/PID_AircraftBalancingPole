# Development Strategy

This project is developed in stages so each layer can be checked independently.

1. Fixed PWM output on PA1/PA2.
2. USART1 heartbeat and diagnostics.
3. Software I2C bus check and MPU6050 register initialization.
4. Raw sensor reads and filtered attitude telemetry.
5. Standalone PID math with output limits and integral anti-windup.
6. A fixed-rate control loop and motor PWM mixing.
7. Calibration and gain tuning on hardware.

The current clock is the STM32F103 HSI at 8 MHz. TIM2 uses a prescaler of 79
and period of 99, giving approximately 1 kHz PWM. The motor direction and
sensor sign are not yet verified, so the PID module is added before it is
allowed to change the PWM outputs.

Until hardware or a working sensor model is available, validation is limited
to compilation, link/HEX generation, deterministic arithmetic checks, and
inspection of the control-loop bounds. Runtime sensor values and PID gains
must be confirmed on the actual assembly.
