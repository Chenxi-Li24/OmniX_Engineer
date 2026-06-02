# Frameworks/bsp_i2c/CMakeLists.txt
cmake_minimum_required(VERSION 3.21)

# 0) List dependencies (other framewornamk libs). Edit to your actual needs.
#    Examples below: fdcan BSP, logging, and motor adaptors.
set(BSP_I2C_DEPS)

# 1) Create library target (static)
add_library(bsp_i2c STATIC
        Inc/bsp_i2c.h)

# 2) Collect sources (auto refresh on changes)
file(GLOB_RECURSE BSP_I2C_SRC CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.S"
)
target_sources(bsp_i2c PRIVATE ${BSP_I2C_SRC})

# 3) Export public include dir
target_include_directories(bsp_i2c
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)

# 4) Inherit MCU flags/defines/includes from the shared interface (mcu_core)
#    This gives Cortex-M7 + FPU flags, HAL/FreeRTOS includes, etc.
target_link_libraries(bsp_i2c PUBLIC mcu_core)

target_include_directories(bsp_i2c
        PUBLIC
        "$<$<TARGET_EXISTS:OmniX::lib_mpu_dcache>:${CMAKE_SOURCE_DIR}/Frameworks/lib_mpu_dcache/Inc>"
)

# 5) Language/feature knobs (safe defaults; keep C/C++混合源码可编译)
#    如果纯C可移除cxx标准；如果需要，解开下面一行：
# target_compile_features(bsp_i2c PUBLIC cxx_std_17)

# 禁止把 memcpy/memmove 优化成可能的字访问序列
target_compile_options(bsp_i2c PRIVATE
        -fno-builtin-memcpy
        -fno-builtin-memmove
        -fno-builtin                # 保险：也别内建其他 libc
        -fno-tree-loop-distribute-patterns
)

# 6) Link required dependent framework libs (transitively, with checks)
foreach(dep IN LISTS BSP_I2C_DEPS)
    if(TARGET ${dep})
        target_link_libraries(bsp_i2c PUBLIC ${dep})
    else()
        message(FATAL_ERROR
                "[bsp_i2c] Required dependency target not found: ${dep}\n"
                " - Check that Frameworks/${dep}/ builds a target named \"${dep}\"\n"
                " - Or update BSP_I2C_DEPS in ${CMAKE_CURRENT_LIST_FILE}")
    endif()
endforeach()

# 7) Optional: module-specific compile definitions / options (don’t leak globally)
# target_compile_definitions(bsp_i2c PUBLIC BSP_I2C_USE_TIM1)
# target_compile_options(bsp_i2c PRIVATE -Wall -Wextra)

# 8) Provide a namespaced ALIAS for convenience
add_library(OmniX::bsp_i2c ALIAS bsp_i2c)

# 9) Export target name upward so the top-level can auto-collect/link it
set(FRAMEWORK_EXPORT_TARGET bsp_i2c PARENT_SCOPE)