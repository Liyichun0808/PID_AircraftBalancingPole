# Isolated Keil source snapshots

Each Keil project under `MDK-ARM` uses its own application snapshot here:

- `Default`
- `AutoStart`
- `Runtime`
- `MotorPwmTest`
- `DirectRun`
- `DirectRunRetry`
- `DirectRunCalibrated`
- `HardwareTest`
- `HardwareI2C`
- `InitFailPwmTest`
- `InitResultPwmTest`
- `WhoAmITest`
- `SensorAxisTest`

Each profile contains independent `Core/Src`, `Core/Inc`, and `MyCode` copies.
Edit the profile directory used by the matching `.uvprojx`; edits to a profile
do not modify another profile. STM32 HAL/CMSIS vendor drivers remain shared.

## DirectRunCalibrated

Use `PID_AircraftBalancingPole_DirectRunCalibrated.uvprojx` when the pole can
be held upright and still during startup. PWM remains at zero while 100 sensor
samples are collected over about one second. The driver averages the gyro bias
and accelerometer angle; the sampled orientation becomes 0 degrees. On
successful calibration, equal base PWM is applied and the DirectRun PID starts
on its first valid control sample. If sensor initialization or calibration
fails, outputs stay at zero. Do not power up with the pole tilted or moving,
because this profile intentionally treats its stationary startup orientation
as the balance reference.

This is the current tuning baseline: all three PID terms are enabled
(`Kp=1.0`, `Ki=0.05`, `Kd=0.08`). The D term uses the calibrated MPU6500
`Gyro Y` rate, while the angle feedback uses the measured `Accel X/Z` axes.
PWM1/PWM2 use opposite correction increments, with the calibrated motor
direction sign fixed in the profile's `correction_sign` setting.

## WhoAmITest

Use `PID_AircraftBalancingPole_WhoAmITest.uvprojx` to identify the sensor over
software I2C on PB6/PB7. It reads register `0x75` at 7-bit addresses `0x68` and
`0x69`, then repeats one PC13 LED pattern: 1 flash for WHO_AM_I `0x68`, 2 for
`0x70`, 3 for `0x71`, 4 for `0x73`, 5 for another acknowledged ID, and 8 when
neither address responds. Each flash is 150 ms on, followed by a 1.5 second
pause after the pattern. This test does not start TIM2 PWM, initialize a motor
controller, or run PID.

## SensorAxisTest

Use `PID_AircraftBalancingPole_SensorAxisTest.uvprojx` to identify the MPU6500
sensor axes without starting either motor PWM or PID. PC13 patterns repeat in
two groups: one marker flash followed by 1/2/3 axis flashes reports the
dominant accelerometer gravity axis; two marker flashes followed by 1/2/3 axis
flashes reports the dominant gyroscope axis during a 2-second motion window.
The axis codes are X=1, Y=2, Z=3. After the axis flashes, a short LED hold means
positive raw value and a long hold means negative. Four flashes mean no
dominant gravity axis or no detected rotation; eight flashes indicate an I2C
or sensor initialization/read error. The live raw samples are also available
in the debugger as `sensor_accel_raw[0..2]` and `sensor_gyro_raw[0..2]`.

## InitResultPwmTest

Use `PID_AircraftBalancingPole_InitResultPwmTest.uvprojx` for the historical
MPU initialization-result check. It starts both PWM outputs at 20%, then runs
the MPU6500 initialization path; successful initialization selects 30% and
failure selects 40%. This standalone profile was reconstructed from the
recorded behavior because its original source revision was not present in the
local Git history.

## Version sequence

The commits before `Archive isolated firmware test and runtime profiles`
preserve the incremental PWM, MPU bring-up, sensor checks, and PID development
history. The separately buildable snapshots now cover the PWM-only test,
MPU initialization result (30%/40%), fail-to-zero diagnostic, PC13 WHO_AM_I
identification, 0x70-specific check, and final software-I2C DirectRun profile.
The 0x70 check is the diagnostic section in the InitFailPwmTest snapshot; the
DirectRun profile uses the same MPU6500 ID and includes the complete PID path.

## DirectRunRetry

Use `PID_AircraftBalancingPole_DirectRunRetry.uvprojx` for the DirectRun
behavior with a short sensor-read grace period. One or two consecutive invalid
samples leave the last PWM command and armed state unchanged; the third
consecutive invalid sample clears both outputs, disarms, and latches a fault.
A valid sample resets the failure counter. The angle-limit protection remains
unchanged. Holding the last command is limited to the existing 10 ms control
period, so this only tolerates brief read glitches; it does not attempt to
continue balancing on invalid sensor data.
