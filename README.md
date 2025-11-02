# Readme
---
This repo is still under development...
This Readme is now only for personal use.

## Project Overview
This is an embedded control system for an autonomous underwater vehicle (AUV) based on the `STM32F103RCTx` microcontroller(for now). 

The Simulation work based on ROS2 and Gazebo is in the repo:
 https://github.com/caneatonly/Acoustic_decoy_ws.git
## Embedded Control Todo List

### 1. System Setup
- [x] Configure STM32 development environment (CubeMX + VS Code + CMake)  
- [x] Initialize project structure and version control with Git  
- [x] Define hardware abstraction layers (sensors, thrusters, valves, communication)

### 2. Core Drivers
- [x] Implement UART/SPI/I2C communication for sensors and modules  
- [x] Develop PWM and ADC drivers for thruster and sensor control  
- [x] Integrate high-pressure gas valve and airbag actuation interfaces  

### 3. RTOS Integration
- [x] Port and configure FreeRTOS  
- [x] Establish task scheduling, queues, and synchronization  
- [x] Implement system tick and timing management  

### 4. Control Modules
- [x] Develop depth estimation using pressure and IMU fusion
    - Currently using EMA filtering; Kalman filter–based fusion of IMU and MS5837 will be developed later.
- [x] Implement PID-based depth and attitude control  
- [ ] SMC and other controllers implementation
- [ ] Design valve control logic for buoyancy adjustment  
- [ ] Integrate fault detection and safety mechanisms  

### 5. Communication & Telemetry
- [x] Implement command/telemetry protocol via UART  
- [x] Add real-time status reporting and parameter tuning interface  

### 7. Future Work
- [ ] Integrate with ROS 2 for high-level control and simulation interface  
- [ ] Explore adaptive and reinforcement-learning-based control strategies
---
### Hardware Components

- **MCU**: STM32F103RCTx (ARM Cortex-M3, 72MHz)
- **IMU**: IM948 (Inertial Measurement Unit)
- **Pressure Sensor**: MS5837 (depth measurement)
- **Actuators**:
  - Motor/propeller for vertical thrust control
  - Electromagnetic lock for nose fairing release
  - Inflation valve for airbag control
- **Debuggers**: ST-Link/V2, nanoDAP wireless CMSIS-DAP

### Software Stack

- **Language**: C (C11 standard)
- **HAL Framework**: STM32CubeMX HAL (Hardware Abstraction Layer)
- **RTOS**: FreeRTOS v10.x
- **Build System**: CMake (≥3.20) + Ninja
- **Toolchain**: arm-none-eabi-gcc

## Development Environment Setup

### Prerequisites

1. **IDE**: Visual Studio Code with STM32 for VS Code extension
2. **Toolchain**: ARM GCC toolchain (`arm-none-eabi-gcc`)
3. **Build Tools**: 
   - CMake (version ≥3.20)
   - Ninja build system
4. **Flash/Debug Tools**:
   - OpenOCD (for nanoDAP wireless)
   - STM32CubeProgrammer (for ST-Link)

### Building the Project

The project uses CMake with predefined presets. Build operations are integrated into VS Code tasks:

```bash
# Configure (automatic with CMake extension)
cmake --preset=Debug

# Build
cmake --build build/Debug

# Or use VS Code task: "CMake: clean rebuild"
```

### Flashing the Firmware

Two flashing methods are available via VS Code tasks:

1. **Using nanoDAP wireless**: Task `"Build + Flash (nanoDAP-wireless)"`
2. **Using ST-Link**: Task `"Build + Flash (ST-Link)"`

Both tasks will:
- Perform a clean rebuild
- Flash the firmware to the target
- Reset and start the MCU
