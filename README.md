# 🤖 OmniX Engineer

> 🏆 **2026 RMUL Engineering Challenge — Shanghai Station Champion**
>
> **RoboMaster 2026 Engineer Robot Firmware** — STM32H723 FreeRTOS 嵌入式固件，驱动 9 轴机械臂 + 4 轮独立转向底盘 + 多 CAN 总线电机协同控制。

**OmniX Engineer** is the embedded firmware for a RoboMaster 2026 competition Engineer robot. Built on STM32H723VGHx (Cortex-M7, hard-float) with FreeRTOS, it controls a 9-axis manipulator, 4-wheel independent swerve drive, and multi-bus CAN motor coordination — all in real-time.

---

## 📁 Project Structure

```
OmniX-Engineer/
├── Core/                     # CubeMX-generated HAL init (USER CODE blocks)
│   ├── Inc/                  # Headers (main.h, fdcan.h, usart.h, gpio.h ...)
│   ├── Src/                  # Sources (main.c, freertos.c, stm32h7xx_it.c ...)
│   └── Startup/              # CMSIS startup assembly
├── Tasks/                    # FreeRTOS application tasks
│   ├── Inc/                  # Task headers
│   └── Src/                  # Task implementations
├── Frameworks/               # Modular reusable components (auto-linked by CMake)
│   ├── bsp_*/                # Board Support Package (hardware abstraction)
│   ├── lib_*/                # Reusable libraries & algorithms
│   ├── lib_adp_*/            # Motor/driver adapters
│   └── tskptt_*/taskptt_*/   # Task prototype modules
├── Applications/             # Higher-level orchestration (extensible)
├── Drivers/                  # STM32CubeH7 HAL & CMSIS (vendor)
├── Middlewares/               # FreeRTOS V10.3.1 kernel
├── openocd_config/           # Debug probe configs (ST-Link, J-Link, CMSIS-DAP)
├── ci/                       # CI build & Feishu notification scripts
└── Docs/                     # Algorithm reports & calibration docs
```

---

## ⚙️ Hardware Target

| Component | Detail |
|-----------|--------|
| **MCU** | STM32H723VGHx (Cortex-M7 @ 550 MHz, hard-float `fpv5-sp-d16`) |
| **RTOS** | FreeRTOS V10.3.1 (CMSIS-RTOS V2) |
| **Toolchain** | `arm-none-eabi-gcc` |
| **Build** | CMake 3.21+ with auto-discovered Frameworks |

### Peripherals

| Peripheral | Usage |
|------------|-------|
| FDCAN1/2/3 | Motor bus communication (500 kbps+) |
| SPI1/2/3 | BMI088 IMU, MCP2518FD external CAN-FD |
| I2C1/2 | EEPROM (24C02), OLED, VT03 camera |
| UART1 | DMA-backed debug logging |
| UART6 | DR16 remote control receiver |
| UART8 | Referee system protocol (921600 bps) |
| UART9 | Serial servo bus (HX-10HM) |
| TIM1/2/3/4/8 | PWM / Motor control |
| OCTOSPI | PSRAM / external flash |
| CORDIC, FMAC | Hardware math accelerators |

---

## 🤖 Robot Subsystems

### 🚗 Chassis — 4-Wheel Independent Swerve Drive

4 个独立转向驱动模块，全向移动。

| Axis | Motor | Bus | Control |
|------|-------|-----|---------|
| 4× Steering | GM6020 ×4 | CAN Bus 1 (IDs 1-4) | Angle PID |
| 4× Drive | C620 ×4 | CAN Bus 2 (IDs 1-4) | Speed PID |

> **Algorithm**: Swerve kinematics → Cascade PID (angle + speed) → Current output

### 🦾 Gimbal — 9-Axis Manipulator

| Joint | Motor | Bus | Mode | Description |
|-------|-------|-----|------|-------------|
| J1–J3 | LK MG8016E ×3 | CAN Bus 1/2/3 | Torque-Position | Main arm joints |
| J4–J7 | DM4310 ×4 | CAN Bus 1/3 | Angle PID (MIT) | Wrist + end-effector |
| J8–J9 | DM4310/DM4340 ×2 | CAN Bus 3 | Torque Assist | J2/J3 gravity compensation |

### 🔫 Shoot

| Motor | Bus | Description |
|-------|-----|-------------|
| C620 ×2 | CAN Bus 2 | Firing wheel + feed |

### 🎯 Sensors

| Sensor | Interface | Purpose |
|--------|-----------|---------|
| BMI088 (×2) | SPI | 6-axis IMU — attitude estimation |
| MCP2518FD | SPI | External CAN-FD controller |
| DR16 Receiver | UART | Remote control (RC input) |
| Referee System | UART (921600 bps) | Competition protocol |
| 24C02 EEPROM | I2C | Calibration persistence |
| VT03 Camera | UART / I2C | Video transmission |

---

## 📋 FreeRTOS Tasks

| Task | Priority | Stack | Role |
|------|----------|-------|------|
| `LED_Task` | Low | 1 KB | WS2812 LED animations |
| `RC_Task` | AboveNormal | 1 KB | DR16 remote control decode |
| `Log_Task` | Low | 4 KB | DMA UART debug logging |
| `Chassis_Task` | High | 4 KB | Swerve drive kinematics + PID |
| `Gimbal_Task` | High | 4 KB | J4–J7 DM motor control |
| `Shoot_Task` | High | 4 KB | Firing mechanism control |
| `Referee_Task` | High | 2 KB | Competition system protocol |
| `Gimbal_behavior` | High | 2 KB | J1–J3 LK motor + J8–J9 compensation |
| `IMU_Task` | High | 2 KB | BMI088 attitude estimation |
| `SerialServo_Task` | Normal | 1 KB | HX-10HM serial bus servo |

---

## 🔧 Build & Flash

### Prerequisites

- `arm-none-eabi-gcc` toolchain (ARM GNU Toolchain)
- CMake ≥ 3.21
- OpenOCD (for flashing/debugging)

### Build

```bash
# Configure
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug

# Compile (produces .elf, .hex, .bin)
cmake --build build

# Check memory footprint
arm-none-eabi-size build/h723vg-v2-freertos.elf
```

### Flash

```bash
# ST-Link probe
openocd -f openocd_config/omxdevh7-stlk.cfg \
  -c "program build/h723vg-v2-freertos.elf verify reset exit"

# J-Link probe
openocd -f openocd_config/omxdevh7-jlnk.cfg \
  -c "program build/h723vg-v2-freertos.elf verify reset exit"
```

---

## 🧩 Frameworks Module System

The `Frameworks/` directory uses a convention-based auto-discovery build system:

```cmake
# CMake auto-discovers all modules in Frameworks/
# Simple modules: Frameworks/<name>/{Inc,Src}/ → auto-linked static lib
# Complex modules: add custom CMakeLists.txt with FRAMEWORK_EXPORT_TARGET
```

| Category | Prefix | Examples |
|----------|--------|----------|
| **BSP** | `bsp_*` | `bsp_fdcan`, `bsp_bmi088`, `bsp_ws2812`, `bsp_buzzer`, `bsp_dr16` |
| **Library** | `lib_*` | `lib_algos`, `lib_imu`, `lib_fault`, `lib_power_control` |
| **Motor Adapter** | `lib_adp_*` | DJI C620, DJI GM6020, DM4310, LK MG8016E, Navision, HX-10HM |
| **Task Prototype** | `tskptt_*` | `tskptt_imu`, `tskptt_exfdcan_router`, `taskptt_ntfdcan_router` |

---

## 📐 Coding Conventions

- **Style**: 2-space indent, K&R braces, PascalCase for tasks
- **Guard**: CubeMX `USER CODE BEGIN/END` blocks
- **Task naming**: `TaskName_Task.h/.c` with `Start_TaskName_Task()` entry
- **Include guards**: `UPPER_SNAKE_CASE`
- **Module prefixes**: `Led_`, `Buzzer_`, `Fdcan_`, etc.
- **Commit format**: Conventional Commits (`feat:`, `fix:`, `chore:`)

---

## 🔗 Dependencies

The `Frameworks/` modules are internal modular libraries designed to be used as **submodules** from an internal OmniX repository. When setting up this project for development:

```bash
# Clone the main repository
git clone https://github.com/Chenxi-Li24/OmniX-Engineer.git
cd OmniX-Engineer

# Initialize framework submodules (requires access to internal repos)
git submodule update --init --recursive
```

> **Note**: Some Frameworks modules reference internal OmniX repositories. Contact the OmniX team for access.

---

## 📄 License

MIT License — see [LICENSE](LICENSE) for details.

> The STM32 HAL/FreeRTOS components in `Drivers/` and `Middlewares/` are distributed under their respective STMicroelectronics and Amazon/FreeRTOS licenses.

---

## 🙏 Acknowledgments

- **OmniX Team** — Hardware design, framework architecture, competition strategy
- **STMicroelectronics** — STM32CubeH7 HAL & CMSIS
- **FreeRTOS** — Real-time kernel
- **DJI RoboMaster** — Competition platform & referee system

---

<p align="center"><i>Built for RoboMaster 2026 · OmniX</i></p>
