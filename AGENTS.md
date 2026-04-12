# AGENTS.md

This file provides guidance to Codex (Codex.ai/code) when working with code in this repository.

**重要提示: 回答全部使用中文，遇到关键词汇附上专业英文翻译**

## 1. 项目架构 (Project Architecture)

本项目是一个基于 STM32F103RCTx 微控制器的声学诱饵 (Acoustic Decoy) 嵌入式系统，用于水下自主航行器 (AUV - Autonomous Underwater Vehicle)。主要功能是通过电机实现定深悬浮，装有充气气囊、高压气瓶等，能够在需要时为气囊充气，AUV上端导流罩 (Fairing) 通过自动机构脱落，气囊在水下展开，实现深度稳定控制。

### 关键技术组件 (Key Technologies)
- **微控制器 (MCU)**: STM32F103RCTx (ARM Cortex-M3)
- **核心框架 (Framework)**: STM32 HAL (Hardware Abstraction Layer)
- **构建系统 (Build System)**: CMake with Ninja generator
- **工具链 (Toolchain)**: ARM GCC (`arm-none-eabi-gcc`)

### 目录结构 (Directory Structure)

- `Acoustic_decoy_v2.ioc`: STM32CubeMX 项目配置文件，**请勿修改**
- `CMakeLists.txt`: 项目顶层 CMake 构建脚本
- `Core/`: CubeMX 生成的核心代码 (HAL初始化、中断处理、系统配置)
  - `Inc/`: 主要头文件，如 `main.h`
  - `Src/`: 主要源文件，如 `main.c`, `stm32f1xx_it.c`
- `Peripherials/`: **自定义外设驱动** (Custom peripheral drivers)
  - IMU948 传感器接口 (`im948_CMD.c/h`)
  - UART BSP 层 (`bsp_usart.c/h`)
  - 传感器数据处理 (`sensor_process.c/h`)
- `Drivers/`: STM32 HAL 库和 CMSIS 核心驱动
- `cmake/`: 构建系统配置和工具链文件

### 核心传感器系统 (Key Sensor Systems)

系统通过 `sensor_process.h` 管理多个传感器和执行器:

#### 传感器模块 (Sensor Modules)
- **IMU948**: 惯性测量单元 (Inertial Measurement Unit)
  - 通信接口: UART2
  - 数据类型: 角度 (angleX/Y/Z) + 加速度 (accelX/Y/Z)
  - 处理函数: `ProcessIMUData()` - FIFO队列处理
- **MS5837**: 压力传感器 (Pressure Sensor) 
  - 通信接口: UART3
  - 数据格式: 字符串 "T=xx.xx D=xx.xx"
  - 测量范围: 温度(-40~85°C), 深度(-10~300m)

#### 执行器控制 (Actuator Control)
- **推进电机 (Propulsion Motor)**: TIM2_CH2 PWM控制 (1000-2000μs)
- **整流罩控制 (Fairing Control)**: TIM3_CH1 伺服控制
- **电磁阀控制 (Solenoid Valve)**: TIM3_CH2 伺服控制
- **12V电源控制**: PA5 (Switch_Pin) 控制MOS开关，统一管理大功率负载

### 硬件依赖 (Hardware Dependencies)

- **主控板 (Main Board)**: STM32F103RCTx开发板
- **调试通信 (Debug Communication)**: HC-05蓝牙模块 (UART1) - 无线调试
- **调试器 (Debugger)**: ST-Link/V2 或 无线调试器CMSIS-DAP (nanoDAP-wireless)
- **传感器 (Sensors)**:
  - IMU948 惯性测量单元 (UART2通信)
  - MS5837 压力传感器 (UART3通信)
- **执行器 (Actuators)**:
  - 推进电调 (ESC) - 12V供电
  - 电磁锁 (Electromagnetic Lock) - 12V供电，整流罩控制
  - 电磁阀 (Solenoid Valve) - 12V供电，气囊充气控制
- **电源管理 (Power Management)**: MOS开关控制12V负载供电

## 2. 开发环境 (Development Environment)

- **IDE**: Visual Studio Code (推荐) + STM32官方VS Code插件
- **编译器工具链 (Toolchain)**: `arm-none-eabi-gcc`
- **构建工具 (Build Tools)**: CMake (>= 3.22), Ninja

## 3. 开发命令 (Development Commands)

### 构建命令 (Build Commands)
```bash
# 配置和构建 (Debug 预设)
cmake --preset Debug
cmake --build --preset Debug

# 清理重新构建
cmake --build --preset Debug --target clean
cmake --build --preset Debug
```

### 烧录/编程 (Flashing/Programming)
支持两种硬件接口:

**nanoDAP-wireless (OpenOCD):**
```bash
# 使用 VS Code 任务: "OpenOCD: Flash project (nanoDAP-wireless)"
# 或运行: "Build + Flash (nanoDAP-wireless)"
```

**ST-Link (STM32CubeProgrammer):**
```bash
# 使用 VS Code 任务: "CubeProg: Flash project (ST-Link)"  
# 或运行: "Build + Flash (ST-Link)"
```

### VS Code 集成 (VS Code Integration)
项目配置了完整的 VS Code 开发环境:
- CMake Tools 扩展集成
- `.vscode/tasks.json` 中的自动化构建和烧录任务
- `.vscode/c_cpp_properties.json` 中的 IntelliSense 配置

## 4. 开发规范 (Development Rules)

### 代码风格 (Coding Style)
- 遵循现有代码的风格（类 K&R 风格）
- 变量和函数命名采用 `snake_case` (蛇形命名法)，例如 `read_sensor_data()`
- 宏定义和枚举成员采用 `UPPER_CASE` (全大写)，例如 `MAX_BUFFER_SIZE`

### 代码注释 (Comments)
- 关键函数需要有 Doxygen 风格的注释，说明函数功能、参数和返回值
- 复杂的逻辑代码块前应有简要的注释说明其作用
- 注释力求简洁明了，避免废话

## 5. 硬件配置详细信息 (Detailed Hardware Configuration)

### 通信接口配置 (Communication Interface Configuration)
- **UART1**: HC-05蓝牙模块，无线调试通信 (`printf` 重定向)
- **UART2**: IMU948传感器数据通信，FIFO缓冲处理
- **UART3**: MS5837压力传感器，字符串数据解析
- **I2C1**: PB6(SCL) + PB7(SDA) - 扩展传感器接口

### 定时器PWM配置 (Timer PWM Configuration)  
- **TIM2_CH2**: 推进电机ESC控制 (1000-2000μs脉宽)
- **TIM3_CH1**: 整流罩伺服控制 (1000/2000μs脉宽)
- **TIM3_CH2**: 电磁阀伺服控制 (1000/2000μs脉宽)
- **TIM3**: ADC1_CH11外部触发源

### GPIO引脚分配 (GPIO Pin Assignment)
- **PA4** (LEDstatus_Pin): 系统状态指示LED
- **PA5** (Switch_Pin): **MOS开关控制** - 12V负载电源管理
- **PA8** (LEDtest_Pin): 测试指示LED  
- **PC4** (BT_status_Pin): 蓝牙连接状态检测 (外部中断)

### 电源管理架构 (Power Management Architecture)
**MOS开关控制方案 (MOS Switch Control Scheme)**:
- 控制信号: PA5 (Switch_Pin)
- 受控负载: 12V供电设备 (电调、电磁锁、电磁阀)
- 优势: 集中电源管理、故障保护、节能设计
- 实现建议: 需要电平转换/驱动电路，添加RC滤波和反向保护

### 完整硬件接口汇总 (Complete Hardware Interface Summary)

| 外设 | 接口/引脚 | 功能 | 通信参数 |
|------|-----------|------|----------|
| HC-05蓝牙 | UART1 | 无线调试 | printf重定向 |
| IMU948传感器 | UART2 | 姿态/加速度 | FIFO缓冲 |  
| MS5837传感器 | UART3 | 温度/深度 | 字符串解析 |
| 推进电调 | TIM2_CH2 | 电机控制 | 1000-2000μs PWM |
| 整流罩控制 | TIM3_CH1 | 机械释放 | 1000/2000μs 脉冲 |
| 电磁阀控制 | TIM3_CH2 | 气囊充气 | 1000/2000μs 脉冲 |
| 电源管理 | PA5 | MOS开关 | 12V负载控制 |
| 状态指示 | PA4/PA8 | LED显示 | 数字输出 |
| 蓝牙状态 | PC4 | 连接检测 | 外部中断 |
| 扩展接口 | I2C1 | 传感器扩展 | PB6/PB7 |
| 模拟采集 | ADC1_CH11 | 电压监测 | TIM3触发 |

## 6. 重要文件 (Important Files)

- `Acoustic_decoy_v2.ioc`: STM32CubeMX 配置文件 - **请勿修改**
- `CMakeLists.txt`: 主构建配置
- `CMakePresets.json`: 构建预设定义
- `GEMINI.md`: 中文项目文档

## 7. 构建目标 (Build Targets)

支持标准的 CMake 构建类型:
- `Debug`: 带调试符号的开发版本
- `Release`: 优化的生产版本  
- `RelWithDebInfo`: 带调试信息的发布版本
- `MinSizeRel`: 大小优化的发布版本

浮点数学库会自动链接 (`-lm` 标志) 以支持传感器计算。