# 基于 PID 算法的双电机平衡杆

## 项目目录说明
`MDK-ARM` 下的每个 Keil 工程都使用这里对应的独立应用源码：

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

每个 profile 都包含独立的 `Core/Src`、`Core/Inc` 和 `MyCode` 副本。请编辑与目标
`.uvprojx` 对应的 profile 目录；修改一个 profile 不会影响其他 profile。STM32
HAL/CMSIS 厂商驱动仍然由各工程共享。

## DirectRunCalibrated：当前调参基线

当平衡杆可以在上电时保持竖直、静止不动时，使用
`PID_AircraftBalancingPole_DirectRunCalibrated.uvprojx`。启动阶段先关闭 PWM，
采集约 1 秒的 100 组传感器数据，计算陀螺仪零偏和加速度计角度平均值，并把上电时
的静止姿态作为 0 度基准。校准成功后输出相同的基础 PWM，并在第一次有效传感器
数据到来时启动 DirectRun PID；传感器初始化或校准失败时保持 PWM 为 0。上电时
不能让平衡杆倾斜或晃动，否则静止姿态会被错误地当成平衡基准。

这是当前后续调参使用的基线：P、I、D 三项全部启用，参数为
`Kp=1.0`、`Ki=0.05`、`Kd=0.08`。D 项使用校准后的 MPU6500 `Gyro Y` 角速度，
角度反馈使用实测的 `Accel X/Z` 轴。PWM1/PWM2 使用方向相反的修正增量，电机
方向由 profile 中的 `correction_sign` 设置固定。

## WhoAmITest：传感器型号识别

使用 `PID_AircraftBalancingPole_WhoAmITest.uvprojx`，通过 PB6/PB7 软件 I2C
识别传感器。程序读取寄存器 `0x75`，依次尝试 7 位地址 `0x68` 和 `0x69`，
然后循环显示 PC13 指示灯结果：WHO_AM_I 为 `0x68` 闪 1 下、为 `0x70` 闪 2 下、
为 `0x71` 闪 3 下、为 `0x73` 闪 4 下、读到其他应答值闪 5 下，两个地址都无应答
闪 8 下。每次亮灯 150 ms，整组闪烁结束后暂停 1.5 秒。本测试不会启动 TIM2 PWM、
初始化电机控制器或运行 PID。

## SensorAxisTest：轴向测试

使用 `PID_AircraftBalancingPole_SensorAxisTest.uvprojx` 测试 MPU6500 轴向，
不会启动两路电机 PWM，也不会运行 PID。PC13 会循环显示两组结果：第一组先闪 1 下
作为标记，再闪 1/2/3 下表示加速度计重力主轴；第二组先闪 2 下作为标记，再闪
1/2/3 下表示 2 秒动作窗口内陀螺仪变化最大的轴。轴编号为 X=1、Y=2、Z=3。轴编号
之后 LED 短暂保持表示原始值为正，长时间保持表示原始值为负；闪 4 下表示没有
明显的重力主轴或旋转，闪 8 下表示 I2C 或传感器初始化/读取错误。调试器中还可以
直接查看实时原始值 `sensor_accel_raw[0..2]` 和 `sensor_gyro_raw[0..2]`。

## InitResultPwmTest：初始化结果测试

使用 `PID_AircraftBalancingPole_InitResultPwmTest.uvprojx` 进行历史版本的 MPU
初始化结果检查。程序先让两路 PWM 输出 20%，然后执行 MPU6500 初始化流程；初始化
成功改为 30%，失败改为 40%。该独立 profile 根据之前记录的运行结果重建，原始源码
版本不在本地 Git 历史中。

## 版本演进

`Archive isolated firmware test and runtime profiles` 之前的提交保留了 PWM 测试、
MPU 连接、传感器检查和 PID 开发过程。当前可独立编译的快照包括：仅 PWM 测试、
MPU 初始化结果（30%/40%）、初始化失败时输出为 0、PC13 WHO_AM_I 识别、`0x70`
专用检查，以及最终的软件 I2C DirectRun 完整 PID 路径。`0x70` 检查位于
`InitFailPwmTest` 快照；DirectRun 使用相同的 MPU6500 型号识别并包含完整 PID 路径。

## DirectRunRetry：短暂读取失败容错

使用 `PID_AircraftBalancingPole_DirectRunRetry.uvprojx` 测试带短暂传感器读取容错的
DirectRun 行为。连续 1 次或 2 次读取无效时，保持上一周期的 PWM 命令和使能状态；
连续第 3 次读取无效时，清零两路输出、撤销使能并锁存故障。有效读取会清零失败计数。
角度限幅保护保持不变。保持上一命令的时间被限制在现有 10 ms 控制周期内，因此它
只用于容忍短暂的读取抖动，不会在传感器无效时继续尝试平衡。
