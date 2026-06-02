# WARP.md

This file provides guidance to WARP (warp.dev) when working with code in this repository.

## Architecture Overview

This is an STM32H723 FreeRTOS-based embedded firmware project with a modular framework architecture:

- **Core/**: CubeMX-generated HAL initialization and configuration (protected by `USER CODE` blocks)
- **Tasks/**: FreeRTOS task implementations following `PascalCase_Task` naming convention  
- **Frameworks/**: Modular, reusable components auto-linked by CMake (BSP drivers, libraries, algorithms)
- **Applications/**: Higher-level features that orchestrate multiple tasks
- **Drivers/**: Vendor CMSIS/HAL sources (STM32CubeH7)
- **Middlewares/**: Third-party components (FreeRTOS kernel)

### Frameworks Module System

The `Frameworks/` directory contains reusable modules that are automatically discovered and linked:

- **BSP modules** (`bsp_*`): Hardware abstraction (buzzer, FDCAN, WS2812, etc.)
- **Library modules** (`lib_*`): Reusable algorithms and data structures  
- **Task modules** (`taskptt_*`): Complex task implementations
- **Adapter modules** (`lib_adp_*`): Hardware-specific driver adapters (DJI motors, etc.)

Modules can either:
1. Use standard `Inc/Src` structure (auto-packaged as static library)
2. Provide custom `CMakeLists.txt` for complex dependencies (see `lib_remote_control`)

## Development Commands

```bash
# Initial setup (ensure arm-none-eabi-* toolchain is in PATH)
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug

# Build firmware (.elf, .hex, .bin outputs)
cmake --build build

# Clean build artifacts
cmake --build build --target clean

# Check memory usage
arm-none-eabi-size build/h723vg-v2-freertos.elf

# Flash to hardware (OpenOCD)
openocd -f openocd_config/stlink.cfg -f openocd_config/stm32h7.cfg -c "program build/h723vg-v2-freertos.elf verify reset exit"
```

### Debug Configurations

Available OpenOCD configs in `openocd_config/`:
- `omxdevh7-stlk.cfg`: ST-Link probe
- `omxdevh7-jlnk.cfg`: J-Link probe  
- `omxdevh7-dap.cfg`: CMSIS-DAP probe

## Code Conventions

- **Indentation**: 2 spaces, K&R braces
- **CubeMX integration**: Keep custom code within `USER CODE BEGIN/END` blocks
- **Task naming**: `TaskName_Task.h/.c` with `Start_TaskName_Task()` entry point
- **Include guards**: Uppercase with underscores (`LED_TASK_H`)
- **Module prefixes**: Use concise tags for internal helpers (`Led_`, `Buzzer_`, etc.)
- **Hard-float**: All modules must use `-mfpu=fpv5-sp-d16` flags (inherited from `mcu_core`)

## Hardware Target

- **MCU**: STM32H723VGHx (Cortex-M7, hard-float)  
- **Linker scripts**: `STM32H723VGHX_FLASH.ld` (production), `STM32H723VGHX_RAM.ld` (debug)
- **FreeRTOS port**: ARM_CM4F (note: uses CM4F port for H7 compatibility)

## Testing & Verification

Currently uses hardware-in-the-loop testing:
- Enable `configASSERT` for runtime validation
- Use ITM/UART traces for debugging (non-blocking to preserve scheduling)
- Test framework placeholder: `Applications/tests/` (when implemented)
- Always verify build and memory usage before commits

## Framework Development

When adding new framework modules:

1. **Simple modules**: Create `Frameworks/module_name/{Inc,Src}/` - auto-linked
2. **Complex modules**: Add custom `CMakeLists.txt` with `FRAMEWORK_EXPORT_TARGET`
3. **Dependencies**: List in `set(MODULE_DEPS ...)` and validate with `if(TARGET ...)`
4. **Naming**: Follow `bsp_*`, `lib_*`, or `taskptt_*` prefixes by category

The build system automatically discovers and links all framework modules to the main executable.