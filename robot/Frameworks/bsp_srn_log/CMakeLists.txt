# Frameworks/bsp_srn_log/CMakeLists.txt
cmake_minimum_required(VERSION 3.21)

# 1) 目标：静态库
add_library(bsp_srn_log STATIC)

# 2) 自动收集源码（包含整个 Src 目录，支持后续新增文件；不把头文件塞进 SOURCES）
file(GLOB_RECURSE BSP_SRN_LOG_SRC CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.cpp"
)
target_sources(bsp_srn_log PRIVATE ${BSP_SRN_LOG_SRC})

# 3) 头文件搜索路径（本模块对外公开 Inc）
target_include_directories(bsp_srn_log
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc
        # 若工程中“存在” lib_mpu_dcache，则把其 Inc 暴露给本模块消费者（生成期判断，顺序无关）
        PUBLIC
        "$<$<TARGET_EXISTS:OmniX::lib_mpu_dcache>:${CMAKE_SOURCE_DIR}/Frameworks/lib_mpu_dcache/Inc>"
)

# 4) 继承 MCU 公共接口（CPU/FPU/优化/全局宏等）
target_link_libraries(bsp_srn_log PUBLIC mcu_core)

# 5) 可选：模块内编译选项/告警（按需开启）
# target_compile_options(bsp_srn_log PRIVATE -Wall -Wextra)
# target_compile_definitions(bsp_srn_log PUBLIC SRN_LOG_ENABLE_DMA=1)

# 6) 命名空间别名（统一风格）
add_library(OmniX::bsp_srn_log ALIAS bsp_srn_log)

# 7) 导出给顶层自动收集
set(FRAMEWORK_EXPORT_TARGET bsp_srn_log PARENT_SCOPE)