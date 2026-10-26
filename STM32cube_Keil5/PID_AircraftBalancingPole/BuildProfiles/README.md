# Isolated Keil source snapshots

Each Keil project under `MDK-ARM` uses its own application snapshot here:

- `Default`
- `AutoStart`
- `Runtime`
- `MotorPwmTest`
- `DirectRun`
- `HardwareTest`
- `HardwareI2C`
- `InitFailPwmTest`
- `WhoAmITest`

Each profile contains independent `Core/Src`, `Core/Inc`, and `MyCode` copies.
Edit the profile directory used by the matching `.uvprojx`; edits to a profile
do not modify another profile. STM32 HAL/CMSIS vendor drivers remain shared.

## WhoAmITest

Use `PID_AircraftBalancingPole_WhoAmITest.uvprojx` to identify the sensor over
software I2C on PB6/PB7. It reads register `0x75` at 7-bit addresses `0x68` and
`0x69`, then repeats one PC13 LED pattern: 1 flash for WHO_AM_I `0x68`, 2 for
`0x70`, 3 for `0x71`, 4 for `0x73`, 5 for another acknowledged ID, and 8 when
neither address responds. Each flash is 150 ms on, followed by a 1.5 second
pause after the pattern. This test does not start TIM2 PWM, initialize a motor
controller, or run PID.
