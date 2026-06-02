# Frameworks/lib_power_control/CMakeLists.txt
cmake_minimum_required(VERSION 3.21)

# 0) 依赖
set(LIB_POWER_CONTROL_DEPS
        bsp_srn_log
        lib_algos
)

# 1) 建库
add_library(lib_power_control STATIC
        Inc/lib_power_control.h)  # 头文件列表可补全

# 2) 收集源码
file(GLOB_RECURSE LIB_POWER_CONTROL_SRC CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.S"
)
target_sources(lib_power_control PRIVATE ${LIB_POWER_CONTROL_SRC})

# 3) 头文件导出
target_include_directories(lib_power_control
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)

# 4) 继承通用 MCU 配置
target_link_libraries(lib_power_control PUBLIC mcu_core)

# 5) 语言/特性（需要 C++17 时解开）
# target_compile_features(lib_power_control PUBLIC cxx_std_17)

# 6) 链接依赖
foreach(dep IN LISTS LIB_POWER_CONTROL_DEPS)
    if(TARGET ${dep})
        target_link_libraries(lib_power_control PUBLIC ${dep})
    else()
        message(FATAL_ERROR "[lib_power_control] Missing dependency target: ${dep}")
    endif()
endforeach()

# 7) 模块自定义编译开关（可选）
# target_compile_definitions(lib_power_control PUBLIC LIB_POWER_CONTROL_USE_SOMETHING)
# target_compile_options(lib_power_control PRIVATE -Wall -Wextra)

# 8) 提供别名（可选）
add_library(OmniX::lib_power_control ALIAS lib_power_control)

# 9) 导出给上层自动收集
set(FRAMEWORK_EXPORT_TARGET lib_power_control PARENT_SCOPE)
