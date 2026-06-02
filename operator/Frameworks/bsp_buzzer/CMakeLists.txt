# Frameworks/bsp_buzzer/CMakeLists.txt
cmake_minimum_required(VERSION 3.21)

# 1) Create library target (static)
add_library(bsp_buzzer STATIC)

# 2) Collect sources
#    Use GLOB for convenience in embedded projects; update on file changes.
file(GLOB_RECURSE BSP_BUZZER_SRC CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.S"
)

target_sources(bsp_buzzer PRIVATE ${BSP_BUZZER_SRC})

# 3) Public include dir = Inc/
#    Consumers will include with: #include "bsp_buzzer.h"
target_include_directories(bsp_buzzer
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)

# 4) Inherit MCU flags/defines/includes from the shared interface (mcu_core)
#    This gives Cortex-M7 + FPU flags, HAL/FreeRTOS includes, etc.
target_link_libraries(bsp_buzzer PUBLIC mcu_core)

target_include_directories(bsp_buzzer
        PUBLIC
        "$<$<TARGET_EXISTS:OmniX::lib_mpu_dcache>:${CMAKE_SOURCE_DIR}/Frameworks/lib_mpu_dcache/Inc>"
)

# 5) (Optional) Module-specific compile definitions / options
#    Put feature toggles here to avoid leaking into whole project.
# target_compile_definitions(bsp_buzzer PUBLIC BSP_BUZZER_USE_TIM1)
# target_compile_options(bsp_buzzer PRIVATE -Wall -Wextra)

# 6) (Optional) If this module needs other libs, link them here.
#    Example: depends on some other framework 'bsp_dma' or 'cmsis_dsp'
# target_link_libraries(bsp_buzzer PUBLIC bsp_dma cmsis_dsp)

# 7) Export target name upward so the top-level can auto-collect/link it
set(FRAMEWORK_EXPORT_TARGET bsp_buzzer PARENT_SCOPE)