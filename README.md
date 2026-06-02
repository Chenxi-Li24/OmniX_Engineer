# 🤖 OmniX Engineer

> 🏆 **2026 RMUL Engineering Challenge — Shanghai Station Champion**
>
> **RoboMaster 2026 Engineer Robot** — STM32H723 dual-firmware embedded system with operator-side custom controller and robot-side 9-axis manipulator + 4-wheel swerve drive, communicating via RS232 / VT03 video link at 921600 bps.

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────┐      0x0302 (30B, 30Hz)     ┌──────────────────────────────┐
│   🎮 选手端 · Operator Controller    │ ──────────────────────────► │   🤖 机器人端 · Robot          │
│   operator/                         │ ◄────────────────────────── │   robot/                      │
│                                      │      0x0309 (30B, 10Hz)     │                              │
│   STM32H723VGHx                     │                              │   STM32H723VGHx               │
│   ┌───────────────────────────┐     │    RS232 over VT03 Video     │   ┌──────────────────────┐    │
│   │ VT03 Receiver (UART9)     │─────│──── Link (921600 Baud) ──────│───│ VT03 TX (UART8)      │    │
│   │ DR16 RC (UART6)           │     │                              │   │                      │    │
│   │ Gimbal + Chassis Tasks    │     │                              │   │ J1-J3: LK MG8016E    │    │
│   │ RC Priority: VT03 > DR16  │     │                              │   │ J4-J7: DM4310/4340   │    │
│   │ JMU Processing            │     │                              │   │ J8-J9: Compensation  │    │
│   │ Power Management          │     │                              │   │ 4× Swerve Drive      │    │
│   └───────────────────────────┘     │                              │   │ Serial Servo (H10HM) │    │
└─────────────────────────────────────┘                              └──────────────────────────────┘
```

---

## 📁 Repository Structure

```
OmniX_Engineer/
├── robot/                   # 🤖 Robot-side firmware (engineer-controller)
│   ├── Core/                # CubeMX HAL initialization
│   ├── Tasks/               # FreeRTOS tasks (10 tasks + CAN routers)
│   ├── Frameworks/          # 28+ BSP/lib/adapter modules (CMake auto-link)
│   ├── Drivers/             # STM32CubeH7 HAL + CMSIS
│   ├── Middlewares/         # FreeRTOS V10.3.1 kernel
│   └── openocd_config/      # Debug probe configs
│
├── operator/                # 🎮 Operator-side custom controller
│   ├── Core/                # CubeMX HAL initialization
│   ├── Tasks/               # FreeRTOS tasks (9 tasks + CAN routers)
│   ├── Frameworks/          # 28+ BSP/lib/adapter modules
│   ├── Docs/                # Algorithm reports & calibration docs
│   └── ci/                  # CI build scripts
│
└── README.md, LICENSE
```

---

## 🔗 Communication Protocol (RS232 / VT03 Video Link)

Both boards communicate through the **RoboMaster Referee System protocol** over UART at 921600 bps.

### Frame Format

```
┌──────┬──────────┬─────┬──────┬────────┬───────────┬────────┐
│ SOF  │ DataLen  │ Seq │ CRC8 │ CmdID  │  Payload  │ CRC16  │
│ 0xA5 │  2 bytes │  1B │  1B  │ 2 bytes│  N bytes  │ 2 bytes│
└──────┴──────────┴─────┴──────┴────────┴───────────┴────────┘
```

### Custom Controller Commands

| CmdID | Direction | Hz | Size | Description |
|-------|-----------|-----|------|-------------|
| `0x0302` | Operator → Robot | 30 Hz | 30B | Joint target positions (J1–J7) |
| `0x0309` | Robot → Operator | 10 Hz | 30B | Joint feedback + online status |

### Joint Mapping (0x0302 Payload)

| Joint | Motor | Range | Description |
|-------|-------|-------|-------------|
| J1 | LK MG8016E | 240° | Base rotation |
| J2 | LK MG8016E | — | Shoulder |
| J3 | LK MG8016E | — | Elbow |
| J4 | DM4340 | — | Arm roll (Big Roll) |
| J5 | DM4310 | — | Wrist yaw |
| J6 | DM4310 | — | Wrist roll |
| J7 | DM4310 | — | Gripper / jaw |

Where raw → degrees conversion: `deg = (raw - zero) × 240.0 ÷ 1000.0`

---

## ⚙️ Hardware Target (Both Boards)

| Component | Detail |
|-----------|--------|
| **MCU** | STM32H723VGHx (Cortex-M7 @ 550 MHz, `fpv5-sp-d16`) |
| **RTOS** | FreeRTOS V10.3.1 (CMSIS-RTOS V2) |
| **Toolchain** | `arm-none-eabi-gcc` |
| **Build** | CMake 3.21+ with auto-discovered Frameworks |
| **FDCAN** | 3× buses (500 kbps+), MCP2518FD external CAN-FD |
| **IMU** | BMI088 ×2 (SPI, 6-axis) |
| **RC** | DR16 (UART6) + VT03 Receiver (UART9) |
| **Debug** | OpenOCD (ST-Link / J-Link / CMSIS-DAP) |

---

## 🤖 Subsystems

### Chassis — 4-Wheel Independent Swerve Drive

| Axis | Motor | Bus | Control |
|------|-------|-----|---------|
| 4× Steering | GM6020 ×4 | CAN1 (ID 1-4) | Angle PID |
| 4× Drive | C620 ×4 | CAN2 (ID 1-4) | Speed PID |

> Algorithm: Swerve kinematics → cascade PID → current output

### Manipulator — 9-Axis Control

| Joint | Motor | Bus | Mode |
|-------|-------|-----|------|
| J1–J3 | LK MG8016E ×3 | CAN1/2/3 | Torque-Position |
| J4–J7 | DM4310/4340 | CAN1/3 | Angle PID (MIT) |
| J8–J9 | DM4310/4340 ×2 | CAN3 | Torque Assist |

---

## 📋 FreeRTOS Tasks

| Task | Priority | Role |
|------|----------|------|
| `RC_Task` | **Realtime7** (operator) | DR16/VT03 input arbitration |
| `Chassis_Task` | High | Swerve drive kinematics |
| `Gimbal_Task` | High | DM J4–J7 control |
| `Gimbal_behavior` | High | LK J1–J3 + J8–J9 compensation |
| `Referee_Task` | High | 0x0302/0x0309 protocol handler |
| `IMU_Task` | High | BMI088 attitude estimation |
| `SerialServo_Task` | Normal (robot only) | HX-10HM serial bus |

---

## 🔧 Build & Flash

### Prerequisites

```bash
# ARM GNU Toolchain
arm-none-eabi-gcc --version
# CMake ≥ 3.21
cmake --version
```

### Build

```bash
# Robot-side firmware
cd robot
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build

# Operator-side firmware
cd ../operator
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
```

### Flash

```bash
# ST-Link
openocd -f robot/openocd_config/omxdevh7-stlk.cfg \
  -c "program robot/build/h723vg-v2-freertos.elf verify reset exit"
```

---

## 🧩 Frameworks Module System

28+ reusable modules auto-discovered by CMake:

| Category | Prefix | Examples |
|----------|--------|----------|
| BSP | `bsp_*` | fdcan, bmi088, ws2812, buzzer, dr16, vt03 |
| Library | `lib_*` | algos (PID/Kalman/Mahony/EKF), imu, fault, power_control |
| Motor | `lib_adp_*` | DJI C620/GM6020, DM4310/4340, LK MG8016E, Navision |
| Serial | `lib_serial_*` | HX-10HM, H12H servo bus |
| CAN | `tskptt_*` / `taskptt_*` | FDCAN/EXFDCAN routers |

Auto-link: `Frameworks/<name>/{Inc,Src}/` → static library → linked to main executable.

---

## 📄 License

MIT — see [LICENSE](LICENSE). STM32 HAL/FreeRTOS components under respective vendor licenses.

## 🙏 Acknowledgments

- **OmniX Team** — Hardware, framework architecture, competition strategy
- **DJI RoboMaster** — Competition platform, referee system, VT03 modules
- **STMicroelectronics** — STM32CubeH7
- **FreeRTOS** — Real-time kernel

---

<p align="center"><i>🏆 RMUL 2026 Shanghai Champion · Built by OmniX</i></p>
