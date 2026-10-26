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

## ProjectDirectRun

Use `PID_AircraftBalancingPole_DirectRun.uvprojx` when the controller should
start without a UART command or gyro bias calibration. It applies the
configured base PWM immediately, initializes and reads the MPU6050, then
starts PID correction on the first valid sample. If the MPU6050 cannot be
initialized or later loses communication, outputs are cleared. Sensor
initialization is required for closed-loop balance control; omitting its
wake/configuration would leave no valid attitude feedback.

## HardwareI2C

Use `PID_AircraftBalancingPole_HardwareI2C.uvprojx` for the same ProjectDirectRun
firmware behavior with only the I2C transport changed to STM32F103 hardware
I2C1. I2C1 runs at 100 kHz on PB6 (SCL) and PB7 (SDA). The matching CubeMX
configuration is `PID_AircraftBalancingPole_HardwareI2C.ioc`. Use external
pull-ups from both bus lines to 3.3 V (typically 4.7 kOhm) if the board or
MPU6050 module does not already provide them. Do not connect these lines to 5 V.

## InitFailPwmTest

Use `PID_AircraftBalancingPole_InitFailPwmTest.uvprojx` to verify that PA1 and
PA2 produce PWM when software-I2C/MPU6050 initialization fails. On failure,
both outputs are set to 50% duty; on success, the ProjectDirectRun path is
unchanged. This does not apply if TIM2/PWM initialization itself fails, because
the timer cannot produce a waveform in that case. Test with motors disconnected
or propellers removed; 50% may start a connected motor.
