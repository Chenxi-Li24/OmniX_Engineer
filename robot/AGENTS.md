# Repository Guidelines

## Project Structure & Module Organization
`Core/Inc` and `Core/Src` carry CubeMX startup logic, HAL glue, and user stubs; keep custom code inside the `USER CODE` guards. `Drivers/` mirrors vendor CMSIS/HAL sources, and `Middlewares/Third_Party/FreeRTOS` stores the RTOS stack. Reusable board support belongs in `Frameworks/`, which the root CMakeLists auto-links. Place FreeRTOS task headers in `Tasks/Inc` and implementations in `Tasks/Src` (see `LED_Task`), while `Applications/` hosts features that orchestrate multiple tasks. Use `openocd_config/` for probe definitions, and let build products accumulate inside `cmake-build-*` directories.

## Build, Test, and Development Commands
- `cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug` — configure once `arm-none-eabi-*` tools are on your `PATH`.
- `cmake --build build` — compile the firmware and emit `.elf`, `.hex`, and `.bin` outputs.
- `cmake --build build --target clean` — drop objects without touching cached settings.
- `arm-none-eabi-size build/h723vg-v2-freertos.elf` — check flash and RAM headroom before merge.
- `openocd -f openocd_config/stlink.cfg -f openocd_config/stm32h7.cfg -c "program build/h723vg-v2-freertos.elf verify reset exit"` — flash and verify on hardware; swap config files to match your probe.

## Coding Style & Naming Conventions
Follow CubeMX defaults: two-space indentation, K&R braces, and single-statement lines. Keep generated blocks intact and extend behavior between matching `USER CODE` comments. Name FreeRTOS task artifacts with PascalCase plus `_Task`; headers use uppercase include guards (`LED_TASK_H`). Prefix internal helpers with a concise module tag (e.g., `Led_`). Apply `clang-format` with the repository profile when it lands; until then, mirror the existing layout manually.

## Testing Guidelines
No automated suite ships yet, so exercise changes on hardware with `configASSERT` enabled and record UART/ITM traces for regression evidence. When you introduce unit or integration tests, stage them under `Applications/tests`, wire them into CMake targets, and document invocation here.

## Commit & Pull Request Guidelines
Use Conventional Commit prefixes (`feat:`, `fix:`, `chore:`) and keep each commit focused. Note build or hardware verification in the footer (`Tested: cmake --build build`). Pull requests should outline subsystem impact, link issues, provide evidence (logs, traces, measurements), and request reviewers responsible for the affected module (`Core`, `Tasks`, `Frameworks`).

## Hardware & Debug Notes
The firmware targets STM32H723 with hard-float (`fpv5-sp-d16`); ensure new modules inherit identical compiler flags. Align linker updates with `STM32H723VGHX_FLASH.ld` or `STM32H723VGHX_RAM.ld`, and favor non-blocking logging (ITM, DMA-backed UART) to keep FreeRTOS scheduling predictable.
