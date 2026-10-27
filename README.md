# 平衡杆 STM32 固件版本

仓库只保留以下六个独立 Keil 工程。每个工程都有自己的源码快照，切换版本不会
互相覆盖。打开对应 `.uvprojx` 编译；Keil 工程依赖的 STM32 HAL/CMSIS 核心驱动
位于 `STM32cube_Keil5/PID_AircraftBalancingPole/Drivers`。

| 序号 | Keil 工程 | 用途 |
|---|---|---|
| 1 | `PID_AircraftBalancingPole_MotorPwmTest.uvprojx` | 不依赖 MPU，测试 TIM2 两路 PWM 输出。 |
| 2 | `PID_AircraftBalancingPole_InitResultPwmTest.uvprojx` | 按 MPU6050（WHO_AM_I `0x68`）初始化；成功输出 30% PWM，失败输出 0%。 |
| 3 | `PID_AircraftBalancingPole_WhoAmITest.uvprojx` | 读取 WHO_AM_I，以 PC13 闪烁次数区分芯片 ID 和 I2C 无应答。 |
| 4 | `PID_AircraftBalancingPole_InitFailPwmTest.uvprojx` | 针对 MPU6500（WHO_AM_I `0x70`）验证寄存器读取；成功输出 30% PWM，失败输出 0%。 |
| 5 | `PID_AircraftBalancingPole_SensorAxisTest.uvprojx` | 不启动 PWM/PID，通过 Keil Watch 查看原始数据或观察 PC13 结果，验证传感器轴向。 |
| 6 | `PID_AircraftBalancingPole_DirectRunCalibrated.uvprojx` | 使用 MPU6500 `0x70` 和实测 XYZ 轴向的完整 PID 程序，是后续调参基线。 |

## 第 6 版调参基线

P、I、D 三项均启用：`Kp=1.0`、`Ki=0.05`、`Kd=0.08`。倾角使用实测的
`Accel X/Z`，角速度反馈使用 `Gyro Y`；`correction_sign` 已按电机实际方向设置。
上电校准约 1 秒，需保持平衡杆竖直且静止；校准时 PWM 为 0，校准成功后才启动
基础 PWM 和 PID。初始化或校准失败时 PWM 保持为 0。

## 安全提示

首次验证电机方向和 PID 时，先拆除螺旋桨并固定杆体。第 2、4 版的 30% PWM 是
固定测试输出，不是平衡控制输出。
