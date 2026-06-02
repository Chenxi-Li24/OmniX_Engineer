# Frameworks/lib_algos/CMakeLists.txt
cmake_minimum_required(VERSION 3.21)

# 1) Create library target (static)
add_library(lib_algos STATIC
        Inc/filters.h
        Inc/swerve_kinematics.h
        Inc/mathutil.h
        Inc/kalman_filter.h
        Inc/pid.h
        Inc/QuaternionEKF.h
        Inc/swerve_kinematics.h)

# 2) Collect sources
#    Use GLOB for convenience in embedded projects; update on file changes.
file(GLOB_RECURSE LIB_ALGOS_SRC CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.S"
)

target_sources(lib_algos PRIVATE ${LIB_ALGOS_SRC})
target_sources(lib_algos PRIVATE
        Src/pid.cpp
)

# 3) Public include dir = Inc/
#    Consumers will include with: #include "lib_algos.h"
target_include_directories(lib_algos
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)

# 4) Inherit MCU flags/defines/includes from the shared interface (mcu_core)
#    This gives Cortex-M7 + FPU flags, HAL/FreeRTOS includes, etc.
target_link_libraries(lib_algos PUBLIC mcu_core)

# 5) (Optional) Module-specific compile definitions / options
#    Put feature toggles here to avoid leaking into whole project.
# target_compile_definitions(lib_algos PUBLIC LIB_ALGOS_USE_TIM1)
# target_compile_options(lib_algos PRIVATE -Wall -Wextra)

# 6) (Optional) If this module needs other libs, link them here.
#    Example: depends on some other framework 'bsp_dma' or 'cmsis_dsp'
# target_link_libraries(lib_algos PUBLIC bsp_dma cmsis_dsp)

# 7) Export target name upward so the top-level can auto-collect/link it
set(FRAMEWORK_EXPORT_TARGET lib_algos PARENT_SCOPE)
