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

Each profile contains independent `Core/Src`, `Core/Inc`, and `MyCode` copies.
Edit the profile directory used by the matching `.uvprojx`; edits to a profile
do not modify another profile. STM32 HAL/CMSIS vendor drivers remain shared.
