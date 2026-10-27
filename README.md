# 基于 PID 算法的双电机平衡杆

面向 STM32F103C8 的固件仓库，保留五个阶段测试版本、一个 P 控制运行基线和一个 Keil Watch 诊断版。
前六个 Keil 工程各自引用独立应用源码；第七版使用独立测试入口，复用第五版 GPIO/I2C 和第六版 Y 轴姿态及控制器。
公共 STM32 HAL、CMSIS 驱动和启动汇编共享。
源码、头文件、启动文件及工程引用的公共驱动均已纳入 Git；不是只上传 `.uvprojx` 工程入口。

## 1. 七个版本及打开入口

所有工程入口位于 `STM32cube_Keil5/PID_AircraftBalancingPole/MDK-ARM/`。
下面的工程和源码链接均指向仓库内的实际路径。

### 版本 1：单独测试 PWM

- 工程：[PID_AircraftBalancingPole_MotorPwmTest.uvprojx](STM32cube_Keil5/PID_AircraftBalancingPole/MDK-ARM/PID_AircraftBalancingPole_MotorPwmTest.uvprojx)。
- 源码：[BuildProfiles/MotorPwmTest](STM32cube_Keil5/PID_AircraftBalancingPole/BuildProfiles/MotorPwmTest)。
- 上电后两路固定输出 20% 占空比，用于验证 TIM2、PA1/PA2 和电机驱动输出。
- 不读取 MPU，不运行 PID；传感器是否连接不影响本测试。

### 版本 2：按 MPU6050 ID 初始化，成功 30%，失败 0%

- 工程：[PID_AircraftBalancingPole_InitResultPwmTest.uvprojx](STM32cube_Keil5/PID_AircraftBalancingPole/MDK-ARM/PID_AircraftBalancingPole_InitResultPwmTest.uvprojx)。
- 源码：[BuildProfiles/InitResultPwmTest](STM32cube_Keil5/PID_AircraftBalancingPole/BuildProfiles/InitResultPwmTest)。
- 保留按 WHO_AM_I 为 `0x68` 判断 MPU6050 的历史检查逻辑；不是 MPU6500 正式运行版。
- 启动时先输出 20% 约 100 ms，再执行传感器初始化；初始化成功改为固定 30%，失败改为 0%。
- 因实际传感器为 MPU6500、ID 为 `0x70`，该版本会因型号不匹配进入失败分支。
- 该快照源于之前重建的初始化结果测试版，当前结果输出按所需历史行为设置为 30%/0%。

### 版本 3：WHO_AM_I 寄存器 LED 识别

- 工程：[PID_AircraftBalancingPole_WhoAmITest.uvprojx](STM32cube_Keil5/PID_AircraftBalancingPole/MDK-ARM/PID_AircraftBalancingPole_WhoAmITest.uvprojx)。
- 源码：[BuildProfiles/WhoAmITest](STM32cube_Keil5/PID_AircraftBalancingPole/BuildProfiles/WhoAmITest)。
- 经 PB6/PB7 软件 I2C 读取寄存器 `0x75`，尝试 7 位设备地址 `0x68`、`0x69`。
- PC13 LED 每组闪烁次数：1 下为 `0x68`，2 下为 `0x70`，3 下为 `0x71`，4 下为 `0x73`，5 下为其他 ID，8 下为总线未就绪或两个地址均未读成功。
- 每次亮 150 ms、灭 250 ms，整组结束后额外停顿 1.5 s；上电读取一次，随后循环显示该结果。
- 不启动电机 PWM，不运行 PID。LED 按低电平点亮配置。

### 版本 4：验证 MPU6500 ID `0x70`

- 工程：[PID_AircraftBalancingPole_InitFailPwmTest.uvprojx](STM32cube_Keil5/PID_AircraftBalancingPole/MDK-ARM/PID_AircraftBalancingPole_InitFailPwmTest.uvprojx)。
- 源码：[BuildProfiles/InitFailPwmTest](STM32cube_Keil5/PID_AircraftBalancingPole/BuildProfiles/InitFailPwmTest)。
- 工程沿用历史名字 `InitFailPwmTest`，实际执行 `main.c` 中的 `SensorId_Read()`，判断读到的 ID 是否为 `0x70`。
- 启动时先输出 20% 约 100 ms；ID 读取成功且匹配后固定输出 30%，读取失败或 ID 不匹配则输出 0%。
- 本版只验证 ID 读取，不运行完整的姿态融合和 PID，也不能代替完整传感器配置验证。
- 本版活动分支不调用快照中旧的 `MPU6050_Init()`，不要仅凭未调用函数中的 `0x68` 判断本版识别目标。

### 版本 5：传感器实际 XYZ 轴向测试

- 工程：[PID_AircraftBalancingPole_SensorAxisTest.uvprojx](STM32cube_Keil5/PID_AircraftBalancingPole/MDK-ARM/PID_AircraftBalancingPole_SensorAxisTest.uvprojx)。
- 源码：[BuildProfiles/SensorAxisTest](STM32cube_Keil5/PID_AircraftBalancingPole/BuildProfiles/SensorAxisTest)。
- 初始化 MPU6500，读取加速度与角速度三轴原始值，不启动电机 PWM，不运行 PID。
- Keil Watch 变量：`sensor_accel_raw[0..2]`、`sensor_gyro_raw[0..2]`，数组下标依次对应 X、Y、Z。
- PC13 第一组先闪 1 下作为加速度标记，再显示重力主轴；第二组先闪 2 下作为角速度标记，再显示 2 秒动作窗口内角速度峰值最大的轴。
- 轴编号 X=1、Y=2、Z=3；轴编号后持续亮 400 ms 表示正值，1400 ms 表示负值；闪 4 下表示未达到主轴/转动判断阈值，闪 8 下表示总线或传感器初始化/读取失败。

### 版本 6：DirectRunCalibrated P 控制版，后续调参基线

- 工程：[PID_AircraftBalancingPole_DirectRunCalibrated.uvprojx](STM32cube_Keil5/PID_AircraftBalancingPole/MDK-ARM/PID_AircraftBalancingPole_DirectRunCalibrated.uvprojx)。
- 源码：[BuildProfiles/DirectRunCalibrated](STM32cube_Keil5/PID_AircraftBalancingPole/BuildProfiles/DirectRunCalibrated)。
- 使用 MPU6500 WHO_AM_I `0x70` 和实测安装轴向，完成姿态估计、P 控制、双电机差动输出和故障关断。
- 上电自动执行初始化、校准及使能，无需串口发送 `A`，主程序不初始化 UART、不使用命令接口。
- 后续调参修改这一版；前五版用于定位硬件和传感器问题。

### 版本 7：Keil Watch 倾角和 P 控制诊断（电机输出关闭）

- 工程：[PID_AircraftBalancingPole_KeilWatchTest.uvprojx](STM32cube_Keil5/PID_AircraftBalancingPole/MDK-ARM/PID_AircraftBalancingPole_KeilWatchTest.uvprojx)。
- 测试源码与操作步骤：[BuildProfiles/KeilWatchTest](STM32cube_Keil5/PID_AircraftBalancingPole/BuildProfiles/KeilWatchTest/README.md)。
- 在 Keil Watch 展开全局 `volatile` 结构体 `g_keil_watch`，可读相对倾角、Y 角速度、P 修正、计算 PWM、实际 CCR 和保护状态。
- 两路实际输出始终为 0，软件控制器正常计算；用 `g_keil_capture_request=1/2/3` 保存竖直、向 PWM1、向 PWM2 的三组数据到 `g_keil_samples[0..2]`。
- 本版按已测通的 SensorAxisTest 启动顺序及编译优化设置组织 GPIO/I2C，保留 DirectRunCalibrated 的实测 Y 轴融合和 P 控制；初始化失败会自动重试，Watch 显示总线电平和尝试次数。
- 本版只检查传感器方向、混控和保护，不测实际推力；未修改两个原版的源码。

> 地址说明：`0x75` 是 WHO_AM_I 寄存器地址，`0x70` 是 MPU6500 返回的芯片 ID，
> **不是 I2C 设备地址**。设备的 7 位通信地址仍为 `0x68` 或 `0x69`，取决于 AD0。
> 软件 I2C 底层还会使用移位后的地址字节，不能把这些常量直接全部替换为 `0x70`。

## 2. 仓库完整目录架构

以下主体展示原六个基线工程的目录结构；新增第七版的入口与共享依赖见版本 7 说明。
本机尚未跟踪的旧工程、仿真和编译产物不属于此结构。
六个应用目录的文件布局相同，在下一节逐一列出公共布局及各文件职责；厂商驱动按实际子目录列出。

```text
PID_AircraftBalancingPole/
├── README.md                              # 中文版本索引、目录架构和使用说明
└── STM32cube_Keil5/
    └── PID_AircraftBalancingPole/
        ├── BuildProfiles/                 # 六套独立应用源码，每套 28 个文件
        │   ├── MotorPwmTest/              # 1. 单独 PWM 测试
        │   ├── InitResultPwmTest/         # 2. MPU6050 0x68：成功 30% / 失败 0%
        │   ├── WhoAmITest/                # 3. LED 识别传感器型号
        │   ├── InitFailPwmTest/           # 4. MPU6500 0x70 ID 验证
        │   ├── SensorAxisTest/            # 5. 实际 XYZ 轴向测试
        │   └── DirectRunCalibrated/       # 6. P 控制运行基线
        ├── Drivers/                       # 六个工程共享的厂商驱动
        │   ├── CMSIS/
        │   │   ├── Include/              # Cortex-M 核心和编译器适配头文件
        │   │   └── Device/ST/STM32F1xx/
        │   │       ├── Include/          # STM32F1 设备寄存器和系统声明
        │   │       ├── LICENSE.txt       # 设备支持文件许可证
        │   │       └── Source/Templates/
        │   │           ├── system_stm32f1xx.c  # 厂商系统文件模板
        │   │           ├── arm/          # ARM 编译器启动模板
        │   │           ├── gcc/          # GCC 启动模板
        │   │           └── iar/          # IAR 启动模板
        │   │               └── linker/   # IAR 链接模板
        │   └── STM32F1xx_HAL_Driver/
        │       ├── Inc/                  # HAL/LL 接口头文件
        │       │   └── Legacy/           # 兼容接口头文件
        │       ├── Src/                  # HAL/LL 厂商驱动源码
        │       └── LICENSE.txt           # HAL 驱动许可证
        └── MDK-ARM/
            ├── startup_stm32f103xb.s     # 六个工程实际共用的启动汇编
            ├── PID_AircraftBalancingPole_MotorPwmTest.uvprojx
            ├── PID_AircraftBalancingPole_InitResultPwmTest.uvprojx
            ├── PID_AircraftBalancingPole_WhoAmITest.uvprojx
            ├── PID_AircraftBalancingPole_InitFailPwmTest.uvprojx
            ├── PID_AircraftBalancingPole_SensorAxisTest.uvprojx
            ├── PID_AircraftBalancingPole_DirectRunCalibrated.uvprojx
            └── DebugConfig/              # 仓库保留的调试配置
                ├── MotorPwmTest_STM32F103C8_1.0.0.dbgconf
                ├── InitFailPwmTest_STM32F103C8_1.0.0.dbgconf
                ├── ProjectSensorAxisTest_STM32F103C8_1.0.0.dbgconf
                └── ProjectDirectRunCalibrated_STM32F103C8_1.0.0.dbgconf
```

### 每个 BuildProfiles 版本内的全部应用文件

下列 `<版本名>` 分别代表前述六个目录，不是需要创建的实际目录。
每套都包含 6 个核心头文件、7 个核心源文件和 15 个功能模块文件，共 28 个文件。版本 6 使用
`p_controller.c/.h` 替代前五套快照中的 `pid_controller.c/.h`。

```text
BuildProfiles/<版本名>/
├── Core/
│   ├── Inc/
│   │   ├── main.h                   # 全局接口、PWM/I2C 等引脚定义
│   │   ├── gpio.h                   # GPIO 初始化接口
│   │   ├── tim.h                    # TIM2 句柄和初始化接口
│   │   ├── usart.h                  # 保留的 USART 接口
│   │   ├── stm32f1xx_hal_conf.h      # 本版本 HAL 模块及晶振参数
│   │   └── stm32f1xx_it.h            # 中断入口声明
│   └── Src/
│       ├── main.c                   # 当前版本入口、时钟和任务流程
│       ├── gpio.c                   # GPIO 模式和初始电平
│       ├── tim.c                    # TIM2 PWM 和引脚复用配置
│       ├── usart.c                  # 保留的 USART 初始化实现
│       ├── stm32f1xx_hal_msp.c       # HAL 底层时钟/引脚支持
│       ├── stm32f1xx_it.c            # 中断处理、SysTick 时间基准
│       └── system_stm32f1xx.c        # 实际编译的系统启动/时钟支持
└── MyCode/
    ├── software_i2c.c               # PB6/PB7 软件 I2C 时序和读写
    ├── software_i2c.h               # 软件 I2C 引脚、宏及接口
    ├── mpu6050.c                    # MPU 寄存器访问、采样、校准和姿态融合
    ├── mpu6050.h                    # MPU 数据类型和寄存器定义
    ├── p_controller.c               # DirectRunCalibrated 使用的 P 控制
    ├── p_controller.h               # P 控制参数和接口
    ├── balance_controller.c         # 使能、故障保护及两路 PWM 混控
    ├── balance_controller.h         # 平衡控制参数、状态及接口
    ├── control_console.c            # 保留的串口命令模块
    ├── control_console.h            # 串口命令接口
    ├── hardware_i2c.c               # 保留的硬件 I2C 实现
    ├── hardware_i2c.h               # 硬件 I2C 接口
    ├── OLED.c                       # 保留的 OLED 驱动
    ├── OLED.h                       # OLED 接口
    └── OLED_Font.h                  # OLED 字库
```

版本 6 使用上面列出的 `p_controller.c/.h`；前五套快照在相同位置使用
`pid_controller.c/.h`，两组文件互相替代，每套仍为 28 个应用文件。

### 文件存在、参加编译和实际运行的区别

- 六套目录保留各自的源码快照，所以测试版目录里也可能含 PID、OLED 等模块文件；不能只凭目录中存在文件推断它会运行。
- `.uvprojx` 的源文件列表决定哪些 `.c` 参加编译，工程宏和调用关系决定哪些代码实际运行。
- `MotorPwmTest` 不把 `MyCode` 源文件加入工程；只执行固定 PWM 分支。
- 第 2、4 版工程列表包含软件 I2C、MPU、PID、平衡控制和命令模块，但当前主程序只执行各自的初始化/ID 检查测试分支。
- 第 3、5、6 版工程列表包含软件 I2C、MPU、控制器和平衡控制模块；第 3、5 版不会进入电机控制流程，第 6 版使用 P 控制。
- 六个工程均没有把 `OLED.c` 或 `hardware_i2c.c` 加入编译；保留这些源文件不表示需要 OLED，也不表示当前使用硬件 I2C。
- `usart.c` 仍在工程源文件列表中；第 6 版没有调用 UART 初始化或命令接口，无需串口输入才能启动。
- 实际编译的系统文件来自对应版本的 `Core/Src/system_stm32f1xx.c`；`Drivers/CMSIS/.../Source/Templates/` 下的是厂商模板。

## 3. 工程与子文件的引用关系

例如打开版本 6 工程时，Keil 按相对路径寻找其源码和头文件：

```text
MDK-ARM/PID_AircraftBalancingPole_DirectRunCalibrated.uvprojx
├── ../BuildProfiles/DirectRunCalibrated/Core/Src/*.c
├── ../BuildProfiles/DirectRunCalibrated/Core/Inc/*.h
├── ../BuildProfiles/DirectRunCalibrated/MyCode/       # 所需功能模块
├── ../Drivers/STM32F1xx_HAL_Driver/Src/              # 工程列入的 HAL 源码
├── ../Drivers/STM32F1xx_HAL_Driver/Inc/
├── ../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy/
├── ../Drivers/CMSIS/Device/ST/STM32F1xx/Include/
├── ../Drivers/CMSIS/Include/
└── startup_stm32f103xb.s
```

其余五个工程分别引用自己同名的 `BuildProfiles/<版本名>`，不会引用另一版的 `main.c`。
应用源码修改相互独立；`Drivers/` 和启动汇编是共享依赖，修改它们可能影响多个版本。
下载或克隆时应保留整个仓库的相对目录结构，不能只复制 `.uvprojx` 或只下载某个 `main.c`。

## 4. 引脚、时钟与 PWM

- 主控：工程设备配置为 STM32F103C8，编译宏为 `STM32F103xB`。
- PWM1：PA1，TIM2_CH2，对应正面偏上的电机。
- PWM2：PA2，TIM2_CH3，对应反面偏下的电机。
- 软件 I2C：PB6 为 SCL，PB7 为 SDA；本仓库保留的六版均使用软件 I2C 路径。
- PC13：第 3、5 版的 LED 指示，低电平点亮。
- PB5：保留的传感器 INT 引脚定义；完整基线通过主循环定时读取传感器，不依靠 MPU 外部中断采样。
- PA9/PA10：保留 USART1 TX/RX 引脚定义，第 6 版运行无需串口。
- 时钟：外部 HSE 按 8 MHz 配置，PLL 倍频 9，系统时钟 72 MHz，APB1 为 36 MHz，TIM2 时钟为 72 MHz。
- TIM2：预分频 `720-1`，自动重装载 `100-1`，PWM 频率为 `72 MHz / 720 / 100 = 1 kHz`。
- 两路输出为占空比可调的 PWM 方波；CCR 写入 20/30 分别对应 20%/30%，CCR 为 0 时没有高电平脉冲。

## 5. DirectRunCalibrated P 控制版的执行架构

```text
上电 / 复位
  → HAL、系统时钟、GPIO、TIM2 初始化，两路占空比先为 0
  → 软件 I2C 总线检查
  → 尝试设备地址 0x68 / 0x69，确认 WHO_AM_I == 0x70
  → 唤醒传感器并写入采样、滤波和量程配置
  → 采集 100 组数据校准陀螺仪零偏与初始姿态，约 1 秒
  → 校准成功：两路先输出基础 PWM 20%，自动请求使能
  → 每 10 ms 读取一次数据：姿态融合 → P 控制 → 保护/混控 → 更新 CCR2/CCR3
```

初始化或校准失败时保持 0% 输出。上电校准期间应让杆体竖直、静止；程序把采样得到的
初始姿态作为零度基准，并不是自动识别真实竖直方向。校准对有效采样数量进行检查，
没有自动证明杆体当时处于竖直或静止状态。

### 姿态估计与控制模块

- `software_i2c.c`：完成 I2C 起止、应答和寄存器读写。
- `mpu6050.c`：虽然文件名保留 MPU6050，版本 6 的识别目标实际为 MPU6500 `0x70`；每次读取加速度 XYZ、温度原始值和角速度 XYZ。
- 倾角采用实测的 X/Z 平面：`atan2f(-AccelZ, -AccelX)` 减去上电校准角度，角速度采用去零偏后的 `Gyro Y`，互补滤波系数为 `0.96`。
- `p_controller.c`：只计算比例项 `u = Kp × (目标角度 − 当前角度)`，再按 `output_limit` 限幅；不累计误差，也不读取角速度作为 D 项。
- `balance_controller.c`：管理自动使能、无效数据计数、倾角保护和两路 PWM 分配。
- `main.c`：组织固定周期采样和模块调用，将控制结果写入 TIM2 硬件比较寄存器。

### PID 三项原理和参数作用

令误差 `e = 目标值 − 测量值`。P、I、D 是三种控制作用，可以单独使用，也可以组合成 PI、PD 或 PID：

- **P（比例）**：`uP = Kp × e`。误差越大，修正越大，直接、响应快；`Kp` 太小会感觉软、回正慢，太大会过冲、来回摆动，也会放大测量噪声。P 控制通常会留下静差，因为误差接近零时修正也接近零。
- **I（积分）**：`uI = Ki × ∫e dt`。累积持续存在的误差，用来消除负载或电机差异造成的静差；`Ki` 太大会累积过头，产生过冲和长时间恢复，输出饱和时还可能积分饱和，因此完整 PID 会做积分限幅和抗饱和。
- **D（微分）**：`uD = Kd × de/dt`。依据误差变化趋势提前制动，可增加阻尼、减少过冲；`Kd` 太大或测量噪声较多时会让输出抖动。此项目完整 PID 快照在目标固定时以负角速度作为误差变化率，避免直接差分带噪的倾角。

这版只使用 P 项，控制修正按比例系数与角度误差计算，不含 Ki/Kd、积分状态或微分项。陀螺仪仍参与姿态互补滤波，但不直接参与控制器的 D 阻尼；相较完整 PID，P-only 更适合逐步确认方向和比例响应，抗扰和抑制摆动能力较弱。

### 当前参数与混控

参数集中在版本 6 的 `Core/Src/main.c` 中的 `controller_config`：

- `target_angle_deg = 0.0`：目标为上电校准得到的零度姿态。
- `kp = 1.0`：比例增益；角度误差单位为度，修正量单位为 PWM 占空比百分点，因此当前增益相当于每偏 1° 给出 1 个百分点的修正，最终仍受输出限幅约束。
- `base_pwm1 = 20.0`、`base_pwm2 = 20.0`：两路独立基础占空比，目标角度下仍能分别调整；校准成功后的启动输出也使用这两个参数。
- `correction_gain1 = 1.0`、`correction_gain2 = 1.0`：两路独立纠偏系数，分别乘在已限幅的 P 修正量上。使用正值保留现有纠偏方向，数值须经实测标定。
- `output_limit = 30.0`：P 修正量限幅为 ±30 个百分点。
- `correction_sign = -1.0`：按已实测的电机增减方向设置。
- `CONTROL_PERIOD_MS = 10`：控制周期；它不进入 P 公式，但决定传感器更新和控制刷新频率。
- `max_angle_cdeg = 4500`：相对校准基准的倾角超过正负 45° 时关断。
- `sensor_failure_limit = 3`：连续 1、2 次无效采样保持上一次输出；第 3 次清零两路并锁存故障，有效采样会清零失败计数。

混控先计算 `correction = correction_sign × P输出`，再分配：

```text
PWM1 = 限幅并取整(base_pwm1 + correction_gain1 × correction)
PWM2 = 限幅并取整(base_pwm2 - correction_gain2 × correction)
```

每路最终范围为 0～99，因此即使基础 PWM 为 20%，某一路也可能在较大修正时单独降到 0。
倾角超限或连续传感器故障会让两路同时归零并锁存；当前自动运行版需要复位后重新启动。
当前默认值保留原有 20% 基础输出和对称差动响应；力矩补偿尚未标定，不表示两路实际推力或力矩相等。

### 不同电机力臂的标定

用户提供的照片标注：绕 Y 轴旋转，上方 PWM1 的力臂 `d1 = 81 mm`，下方 PWM2 的力臂 `d2 = 71 mm`。
以下关系以照片中的 O 为实际转轴、两路推力分别沿相反 Z 方向为前提；距离须为转轴到推力作用线的垂距。

相同推力时，上方电机的力矩约为下方的 `81 / 71 = 1.1408` 倍。忽略其他外力矩时，目标姿态下两路
平衡要求 `81 × F1 = 71 × F2`，即 `F1 / F2 = 71 / 81 = 0.8765`。这是推力比，不能直接作为 PWM 占空比比值。
重心偏置、摩擦、推力方向、电机和桨的差异仍需实测排查，不能仅凭此尺寸认定单侧无法回正的原因。

- 先拆桨，在校准姿态及分别向两侧倾斜约 5～10° 时，记录 `balance_controller.angle_cdeg`、`pwm1`、`pwm2`、`faulted`，确认角度符号和纠偏命令会随倾斜方向翻转。
- 在固定且有防护的测试架上，记录相同供电电压下各电机在多个工作点的实际推力，确定各自的 PWM—推力关系。
- 基础输出使用 `base_pwm1/base_pwm2` 标定，使目标姿态附近的净力矩平衡；只改变纠偏系数不会改变角度误差为零时的基础输出。
- 若工作点附近推力斜率为 `k1 = dF1/dPWM1`、`k2 = dF2/dPWM2`，两路等力矩增量的纠偏分配满足 `correction_gain1 / correction_gain2 = (d2 × k2) / (d1 × k1)`。只有两路局部斜率相等时，系数比才等于 `71/81`。
- 单侧无法回正时还要检查保护和输出饱和：当前默认配置的单路最大输出为 50%，超过 ±45° 或连续三次传感器失败会两路关断并锁存故障。增大 Kp 不会抬高既有输出限幅。

## 6. 编译与烧录

1. 下载整个仓库，保持 `BuildProfiles/`、`Drivers/` 和 `MDK-ARM/` 的相对位置。
2. 安装 Keil MDK，并提供工程所需的 STM32F1 设备支持及 ARM Compiler 5；当前工程记录的编译器为 V5.06 update 7。
3. 在 `MDK-ARM/` 打开需要的 `.uvprojx`；完整功能和后续调参请选择第 6 版 `DirectRunCalibrated`。
4. 执行 Rebuild，确认使用的是所选版本自己的源文件与输出目录。
5. 在 Keil 中选择实际使用的下载器并设置调试/Flash 下载，编译后烧录到 PCB。
6. 运行 P 控制版时，上电先保持杆体竖直静止完成校准，再进行方向检查和参数调整；逐步增加 `kp`，确认输出方向正确后再观察能否稳定。

六个工程的输出目录分别为 `MotorPwmTest/`、`InitResultPwmTest/`、`ProjectWhoAmITest/`、
`InitFailPwmTest/`、`ProjectSensorAxisTest/`、`ProjectDirectRunCalibrated/`，均位于 `MDK-ARM/` 下。
第七版诊断工程输出到 `ProjectKeilWatchTest/`。
这些是编译时生成的目录；仓库不提交 HEX、AXF、OBJ、MAP、日志或个人 Keil 窗口设置。
当前仓库不包含 CubeMX `.ioc` 和 Proteus 仿真文件，使用已保存的 Keil 工程编译。

## 7. 上传完整性与检查范围

以下为增加 Keil Watch 诊断版之前，2026-10-27 对 GitHub `master` 原六个工程所做的历史检查：

- 六个 `.uvprojx` 工程均已上传且可解析。
- 六套应用源码各 28 个文件，合计 168 个，包含所有应用 `.c`/`.h` 和字库头文件。
- 六个工程源文件列表合计 154 项引用（含跨工程重复引用），对应文件均在 GitHub 文件树中。
- 六个工程各配置 6 个头文件搜索目录，合计 36 项引用，对应目录均在 GitHub 中。
- 从工程源文件递归检查双引号 `#include` 的本地头文件依赖，没有发现仅本机存在而 GitHub 缺失的依赖；标准库头文件由编译器提供。
- HAL/CMSIS 公共驱动、实际引用的启动汇编和保留的调试配置均已上传。
- 本次核验确认的是上传完整性和引用路径；不等同于重新通过 Keil 编译或重新完成实物验证。

本次 README 补充记录了固件 P 控制改动，没有改变引脚或其他硬件配置。首次检查电机方向时，
应先拆除螺旋桨并固定杆体；第 1、2、4 版可能在启动时直接输出固定 PWM。
