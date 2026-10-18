# Firmware profiles

Both profiles use the same sensor, PID, safety, and serial-control code.

## HardwareTest

Use the original Keil project or
`PID_AircraftBalancingPole_HardwareTest.uvprojx`. It builds a HEX with
`output_enabled=0`. It is for checking power, clock, UART, I2C, MPU6050
`WHO_AM_I`, calibration, and telemetry. `A` records an arm request but cannot
energize the motor PWM outputs.

## ProjectRuntime

Use `PID_AircraftBalancingPole_Runtime.uvprojx`. It defines
`PROJECT_RUNTIME_BUILD`, so `output_enabled=1`. The PWM pins remain at zero
after reset; output is possible only after a valid MPU6050 startup, 50 valid
10 ms samples, one second elapsed, and an `A` command. `D`, an angle outside
`+/-45` degrees, or three failed samples immediately clears both outputs.

The runtime profile is intended for the assembled project after motor driver
polarity, sensor axis, base PWM, and PID gains have been checked. Start with
the HardwareTest HEX on a new PCB.

## MotorPwmTest

Use `PID_AircraftBalancingPole_MotorPwmTest.uvprojx` to check the PWM pins
without an MPU6050. After reset it outputs fixed 1 kHz, 20% duty PWM on PA1
(TIM2_CH2) and PA2 (TIM2_CH3). The profile does not initialize or use the
MPU6050, I2C, PID, or UART application paths. Remove propellers and loads
before powering any connected motor; connect motors only through the motor
driver, never directly to STM32 pins.
