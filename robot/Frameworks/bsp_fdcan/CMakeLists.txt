# Frameworks/bsp_fdcan/CMakeLists.txt
cmake_minimum_required(VERSION 3.21)

# 1) 目标
add_library(bsp_fdcan STATIC)

# 2) 自动收集源码
file(GLOB_RECURSE BSP_FDCAN_SRC CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.cpp"
)
target_sources(bsp_fdcan PRIVATE ${BSP_FDCAN_SRC})

# 3) 公开头文件目录（本模块）+ 条件加入 lib_mpu_dcache 的 Inc（若存在）
target_include_directories(bsp_fdcan
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc
        "$<$<TARGET_EXISTS:OmniX::lib_mpu_dcache>:${CMAKE_SOURCE_DIR}/Frameworks/lib_mpu_dcache/Inc>"
)

# 4) 继承 MCU 公共接口
target_link_libraries(bsp_fdcan PUBLIC mcu_core)

# 5) 可选：模块内编译选项/宏（按需启用）
# target_compile_options(bsp_fdcan PRIVATE -Wall -Wextra)
# target_compile_definitions(bsp_fdcan PUBLIC BSP_FDCAN_ENABLE_RXFIFO1=1)

# 6) 命名空间别名
add_library(OmniX::bsp_fdcan ALIAS bsp_fdcan)

# 7) 导出给顶层自动收集
set(FRAMEWORK_EXPORT_TARGET bsp_fdcan PARENT_SCOPE)