# Frameworks/tskptt_imu/CMakeLists.txt
cmake_minimum_required(VERSION 3.21)

# 0) List dependencies (other framework libs). Edit to your actual needs.
#    Examples below: fdcan BSP, logging, and motor adaptors.
set(TSKPTT_IMU_DEPS
        lib_imu
        bsp_srn_log
        lib_24c02
        bsp_i2c
        tskptt_ntfdcan_router
)

# 1) Create library target (static)
add_library(tskptt_imu STATIC
        Inc/tskptt_imu.h
)

# 2) Collect sources (auto refresh on changes)
file(GLOB_RECURSE TSKPTT_IMU_SRC CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.S"
)
target_sources(tskptt_imu PRIVATE ${TSKPTT_IMU_SRC})

# 3) Export public include dir
target_include_directories(tskptt_imu
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)

# 4) Inherit MCU flags/defines/includes from the shared interface (mcu_core)
#    This gives Cortex-M7 + FPU flags, HAL/FreeRTOS includes, etc.
target_link_libraries(tskptt_imu PUBLIC mcu_core)

# 5) Language/feature knobs (safe defaults; keep C/C++混合源码可编译)
#    如果纯C可移除cxx标准；如果需要，解开下面一行：
# target_compile_features(tskptt_imu PUBLIC cxx_std_17)

# 6) Link required dependent framework libs (transitively, with checks)
foreach(dep IN LISTS TSKPTT_IMU_DEPS)
    if(TARGET ${dep})
        target_link_libraries(tskptt_imu PUBLIC ${dep})
    else()
        message(FATAL_ERROR
                "[tskptt_imu] Required dependency target not found: ${dep}\n"
                " - Check that Frameworks/${dep}/ builds a target named \"${dep}\"\n"
                " - Or update TSKPTT_DEPS in ${CMAKE_CURRENT_LIST_FILE}")
    endif()
endforeach()

# 7) Optional: module-specific compile definitions / options (don’t leak globally)
# target_compile_definitions(tskptt_imu PUBLIC TSKPTT_IMU_USE_TIM1)
# target_compile_options(tskptt_imu PRIVATE -Wall -Wextra)

# 8) Provide a namespaced ALIAS for convenience
add_library(OmniX::tskptt_imu ALIAS tskptt_imu)

# 9) Export target name upward so the top-level can auto-collect/link it
set(FRAMEWORK_EXPORT_TARGET tskptt_imu PARENT_SCOPE)
