# Keil Watch 倾角与 P 控制测试

打开 `MDK-ARM/PID_AircraftBalancingPole_KeilWatchTest.uvprojx`，目标为
`ProjectKeilWatchTest`，输出目录为 `MDK-ARM/ProjectKeilWatchTest/`。

这版用于定位“两侧倾斜时指令是否反向、是否触发保护、某路是否降为零”。
它按已在实物测通的 SensorAxisTest 组织 GPIO、软件 I2C 和传感器启动，
保留 DirectRunCalibrated 的实测 Y 轴姿态算法和 P 控制器，有独立的 main.c 和输出文件。
两路实际 CCR 始终写 0，电机不转；软件控制器仍正常使能，
因此可以读到计算 PWM。不能用这版测实际推力或验证带桨回正。

## 操作步骤

1. 先断开电机动力电源或拆桨，接好调试器，给主控和传感器供电。
2. Keil 选择实际下载器及 SWD 接口，Rebuild 后 Download。
3. 进入 `Debug -> Start/Stop Debug Session`，打开 `View -> Watch Windows -> Watch 1`。
4. 在 Watch 1 添加 `g_keil_watch`、`g_keil_capture_request`、
   `g_keil_capture_result` 和 `g_keil_samples`，展开结构体；数值优先使用十进制。
5. 将杆保持在目标平衡姿态静止，Reset 后点击 Run（F5），保持约 2 秒完成初始化和校准。
   启动先等待 200 ms，再按 SensorAxisTest 的 GPIO → PC13 LED → 软件 I2C → MPU 顺序
   检查与初始化，首次传感器初始化尝试完成后才初始化 TIM2。此前 PA1/PA2 已作为普通输出拉低。
   初始化失败会等待 500 ms 再尝试；重试和重新校准期间仍需保持竖直静止。
   上电姿态会被当作零度，不能斜着校准。PC13 LED 在校准成功后常亮。
6. 正常时 `stage=4`、`mpu_ok=1`、`calibrated=1`、`sample_valid=1`、
   `armed=1`、`faulted=0`，`sample_count` 持续增加。
   `armed=1` 仅表示软件计算已使能，`motors_enabled=0`、两路 `pwm*_actual=0`。
7. 优先在运行中读取 Watch；若数值不刷新，启用 Keil 的 `View -> Periodic Window Update`，
   并确认调试器支持运行中读内存。也可短暂暂停读取，恢复 Run 后等约 1 秒让姿态重新收敛。
   不要边暂停边移动杆，暂停期间传感器不会被软件连续采样。

## 先看这些字段

- `angle_deg`：相对启动校准姿态的倾角，单位度；`angle_cdeg / 100` 为同一量的整数表示。
- `gyro_dps`：去零偏后的 Y 角速度，单位 °/s；`gyro_mdps / 1000` 为整数表示。
- `error_deg`：目标角度减当前倾角，目标为 0°。
- `correction_pwm`：限幅并乘方向符号后的 P 修正，单位 PWM 百分点；
  `correction_milli / 1000` 为同一量。
- `pwm1_calc/pwm2_calc`：经过保护、混控、0～99 限幅和取整的计算指令，1 对应 1% 占空比。
- `pwm1_actual/pwm2_actual`：TIM2 的 CCR2/CCR3 读回值，这个测试中必须始终为 0。
- `faulted/fault_reason`：故障锁存及原因；原因 1 为倾角超限，2 为传感器无效数据，3 为其他。
- `sensor_failure_count`：当前连续无效数据次数；`read_failure_total` 是累计读取失败次数。
- `sample_interval_ms`：相邻采样开始的时间差；`read_duration_ms` 为一次读取耗时。
  原始加速度和角速度保留在 `*_raw` 中，读取失败时保留上一组数据，以 `sample_valid` 区分。
- `init_attempts`：已尝试初始化的次数；`last_init_error`：最近一次初始化失败的阶段码，
  成功重试后仍保留供追查，以当前 `stage` 和 `sample_valid` 判断现在是否正常。
- `scl_high/sda_high`：PB6/PB7 当前读回电平，1 为高、0 为低，在每次采样结束或等待重试时更新。
  在 `stage=100` 时如果持续有一路为 0，继续检查对应线路、模块供电和上拉。
- `sensor_accel_raw[0..2]`、`sensor_gyro_raw[0..2]`：与 SensorAxisTest 同名的全局原始 XYZ 数据，
  每次有效读取更新，可直接加入 Watch；它们未去零偏，不能将原始陀螺仪数值当作 °/s。

当前参数与 DirectRunCalibrated 试调值一致：`Kp=1`、基础 PWM1/PWM2=`17.5/20`、
纠偏系数=`0.8765/1.0`、`correction_sign=-1`、P 修正限幅 ±30。
因此软件角度约 +10° 时计算指令约为 PWM1=26、PWM2=10；约 -10° 时约为 9、30。
这里的 0.8765 是按 `71/81` 得到的力臂补偿起始值，实际输出仍关闭。
这是软件符号关系，向哪一个实体电机倾斜对应正角度，必须通过下面的三次测量确认。
不要仅凭计算值更大就认定实测推力或力矩一定更大。

## 保存三组姿态，避免抄数时不同步

每个姿态保持约 1 秒，再修改 `g_keil_capture_request` 的 Value：

1. 杆竖直：写入 `1`，保存到 `g_keil_samples[0]`。
2. 向 PWM1 的旋桨方向倾斜约 5～10°：写入 `2`，保存到 `g_keil_samples[1]`。
3. 向 PWM2 的旋桨方向倾斜约 5～10°：写入 `3`，保存到 `g_keil_samples[2]`。

在 Run 状态写值；若下载器不能运行中写内存，可暂停设置，恢复 Run 后立即读取下一次采样保存结果。
请求被处理后自动归零，`g_keil_capture_result=1` 表示保存成功，每组 `snapshot_valid=1`。
重复写同一编号可覆盖该槽。结果 2 表示编号不是 1～3；3 表示当前数据无效、尚未使能或已故障，
不会覆盖旧快照。快照存在 RAM，复位或断电会清除。

把三组快照中的 `angle_deg`、`gyro_dps`、`correction_pwm`、`pwm1_calc`、`pwm2_calc`、
`faulted`、`fault_reason`、`sensor_failure_count` 发回来，并注明每组对应的物理倾斜方向。
如捕获失败，同时提供实时 `g_keil_watch.stage` 和 `g_keil_capture_result`。

## 状态和故障

- `stage=0/1/2/3`：启动 / 检查 I2C / 初始化传感器 / 校准。
- `stage=4`：正在采样和计算；`stage=5`：控制器保护已锁存，仍继续读取传感器。
- `stage=100`：I2C 总线未就绪；101：MPU6500 初始化失败；102：校准失败；103：时钟或 TIM 初始化失败。
- 100/101/102 时会自动重试初始化，不会跳过 I2C 高电平检查或芯片 ID 检查。
  如果 `stage=100`、`sample_count=0`，当前倾角和计算 PWM 的零值不是有效测量。
- 相对初始姿态超过 ±45°，或连续三次无效数据，会锁存故障并清零计算 PWM；需要 Reset，
  重新保持目标姿态完成校准。前两次无效采样可能保留上一组计算输出，与运行版逻辑一致。
- 自动重试只适用于启动初始化失败，不会解除运行中的 `stage=5` 保护锁存。
- 这里显示的是已有控制器的计算结果；电机输出始终关闭，也没有可在 Watch 中打开电机的开关。

## 文件依赖

本目录只保存测试入口和 Watch 类型定义。GPIO、软件 I2C、中断、MSP、系统文件以及核心头文件
来自 `BuildProfiles/SensorAxisTest/`，工程采用与 SensorAxisTest 相同的编译优化级别。
TIM2、MPU 姿态驱动和 P 控制器来自 `BuildProfiles/DirectRunCalibrated/`。
两版 MPU 的设备地址探测、WHO_AM_I 检查、寄存器初始化和原始读取代码一致；
SensorAxisTest 保留的是旧 X 轴姿态接口，而当前硬件已确认绕 Y 轴旋转，因此这里保留实测 Y 轴融合。
所有 Watch 变量保持全局 `volatile`，供优化后读取。两个原有版本的源码不变。
厂商驱动与启动汇编也按原工程共享。
应保留整个仓库的目录结构，不能只复制本目录或 `.uvprojx`。
