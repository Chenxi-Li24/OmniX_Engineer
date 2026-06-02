# Frameworks/lib_imu/CMakeLists.txt
cmake_minimum_required(VERSION 3.21)

# 0) List dependencies (other framework libs). Edit to your actual needs.
#    Examples below: fdcan BSP, logging, and motor adaptors.
set(LIB_IMU_DEPS
        bsp_spi
        bsp_bmi088
        lib_algos
)

# 0.5) IMU orientation preset (matches lib_imu_orientation.c)
#     0: identity (sensor frame == body frame)
#     1: flip around X (Bx=+Sx, By=-Sy, Bz=-Sz)
#     2/3/4/5: see lib_imu_orientation.c comments
set(IMU_ORIENTATION_SEL 6 CACHE STRING "IMU orientation preset index" FORCE)

# 1) Create library target (static)
add_library(lib_imu STATIC
        Inc/lib_imu.h
        Inc/lib_ins.h
        Inc/lib_imu_orientation.h)

# 2) Collect sources (auto refresh on changes)
file(GLOB_RECURSE LIB_IMU_SRC CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.S"
)
target_sources(lib_imu PRIVATE ${LIB_IMU_SRC})

# 3) Export public include dir
target_include_directories(lib_imu
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)

# 4) Inherit MCU flags/defines/includes from the shared interface (mcu_core)
#    This gives Cortex-M7 + FPU flags, HAL/FreeRTOS includes, etc.
target_link_libraries(lib_imu PUBLIC mcu_core)

target_include_directories(lib_imu
        PUBLIC
        "$<$<TARGET_EXISTS:OmniX::lib_mpu_dcache>:${CMAKE_SOURCE_DIR}/Frameworks/lib_mpu_dcache/Inc>"
)

# 5) Language/feature knobs (safe defaults; keep C/C++ mixed source compilable)
#    If pure C, you can remove C++ standard; if needed, uncomment:
# target_compile_features(lib_imu PUBLIC cxx_std_17)

# 5.5) Module-specific compile definitions
#      Export IMU_ORIENTATION_SEL to this target
target_compile_definitions(lib_imu
        PUBLIC
        IMU_ORIENTATION_SEL=${IMU_ORIENTATION_SEL}
)

# 禁止把 memcpy/memmove 优化成可能的字访问序列
target_compile_options(lib_imu PRIVATE
        -fno-builtin-memcpy
        -fno-builtin-memmove
        -fno-builtin                # Safety: also disable other libc builtins
        -fno-tree-loop-distribute-patterns
)

# 6) Link required dependent framework libs (transitively, with checks)
foreach(dep IN LISTS LIB_IMU_DEPS)
    if(TARGET ${dep})
        target_link_libraries(lib_imu PUBLIC ${dep})
    else()
        message(FATAL_ERROR
                "[lib_imu] Required dependency target not found: ${dep}\n"
                " - Check that Frameworks/${dep}/ builds a target named \"${dep}\"\n"
                " - Or update LIB_IMU_DEPS in ${CMAKE_CURRENT_LIST_FILE}")
    endif()
endforeach()

# 7) Optional: more module-specific compile definitions / options (do not leak globally)
# target_compile_definitions(lib_imu PUBLIC LIB_IMU_USE_TIM1)
# target_compile_options(lib_imu PRIVATE -Wall -Wextra)

# 8) Provide a namespaced ALIAS for convenience
add_library(OmniX::lib_imu ALIAS lib_imu)

# 9) Export target name upward so the top-level can auto-collect/link it
set(FRAMEWORK_EXPORT_TARGET lib_imu PARENT_SCOPE)