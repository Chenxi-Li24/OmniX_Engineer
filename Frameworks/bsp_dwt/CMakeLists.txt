# Frameworks/bsp_dwt/CMakeLists.txt
cmake_minimum_required(VERSION 3.21)

# 1) Create library target (static)
add_library(bsp_dwt STATIC
            Inc/bsp_dwt.h)

# 2) Collect sources
#    Use GLOB for convenience in embedded projects; update on file changes.
file(GLOB_RECURSE BSP_DWT_SRC CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.S"
)

target_sources(bsp_dwt PRIVATE ${BSP_DWT_SRC})

# 3) Public include dir = Inc/
#    Consumers will include with: #include "bsp_dwt.h"
target_include_directories(bsp_dwt
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)

# 4) Inherit MCU flags/defines/includes from the shared interface (mcu_core)
#    This gives Cortex-M7 + FPU flags, HAL/FreeRTOS includes, etc.
target_link_libraries(bsp_dwt PUBLIC mcu_core)

# 5) (Optional) Module-specific compile definitions / options
#    Put feature toggles here to avoid leaking into whole project.
# target_compile_definitions(bsp_dwt PUBLIC BSP_DWT_USE_TIM1)
# target_compile_options(bsp_dwt PRIVATE -Wall -Wextra)

# 6) (Optional) If this module needs other libs, link them here.
#    Example: depends on some other framework 'bsp_dma' or 'cmsis_dsp'
# target_link_libraries(bsp_dwt PUBLIC bsp_dma cmsis_dsp)

# 7) Export target name upward so the top-level can auto-collect/link it
set(FRAMEWORK_EXPORT_TARGET bsp_dwt PARENT_SCOPE)
