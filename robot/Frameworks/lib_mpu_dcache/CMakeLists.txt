# Frameworks/lib_mpu_dcache/CMakeLists.txt
cmake_minimum_required(VERSION 3.21)

# 0) Module deps (按需填)
set(LIB_MPU_DCACHE_DEPS
        # bsp_srn_log
)

# 0.1) Linker fragment: source & build output (absolute)
set(DMA_NC_LD_SRC "${CMAKE_CURRENT_LIST_DIR}/Lnk/dma_nc.ld")
if(NOT EXISTS "${DMA_NC_LD_SRC}")
    message(FATAL_ERROR "[lib_mpu_dcache] dma_nc.ld not found at: ${DMA_NC_LD_SRC}")
endif()
set(DMA_NC_LD_BIN "${CMAKE_CURRENT_BINARY_DIR}/dma_nc.ld")

# 0.2) Copy rule (absolute OUTPUT avoids 'no rule to make target')
add_custom_command(
        OUTPUT  "${DMA_NC_LD_BIN}"
        COMMAND "${CMAKE_COMMAND}" -E make_directory "${CMAKE_CURRENT_BINARY_DIR}"
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${DMA_NC_LD_SRC}" "${DMA_NC_LD_BIN}"
        DEPENDS "${DMA_NC_LD_SRC}"
        COMMENT "[lib_mpu_dcache] copy dma_nc.ld -> ${DMA_NC_LD_BIN}"
        VERBATIM
)

# 0.3) A phony target that actually materializes the fragment
add_custom_target(lib_mpu_dcache_linker_fragment
        DEPENDS "${DMA_NC_LD_BIN}"
)

# 0.4) 一个极小的“顺序守卫”静态库，确保任何链接到本模块的可执行，
#      都会先触发 linker_fragment，再进入最终链接（不改顶层）
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/_mpu_dcache_order_dummy.c"
        "/* auto-generated; do not edit */
void __mpu_dcache_order_dummy(void) {}
")
add_library(lib_mpu_dcache_order STATIC
        "${CMAKE_CURRENT_BINARY_DIR}/_mpu_dcache_order_dummy.c"
)
# 关键：顺序守卫库 依赖 片段生成
add_dependencies(lib_mpu_dcache_order lib_mpu_dcache_linker_fragment)

# 1) 本模块静态库
add_library(lib_mpu_dcache STATIC
        Inc/mem_sections.h
)

# 2) 自动收集源码
file(GLOB_RECURSE lib_mpu_dcache_SRC CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.S"
)
target_sources(lib_mpu_dcache PRIVATE ${lib_mpu_dcache_SRC})

# 3) 头文件
target_include_directories(lib_mpu_dcache
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)

# 4) MCU 公共接口
target_link_libraries(lib_mpu_dcache PUBLIC mcu_core)

# 5) 其它依赖（存在性检查）
foreach(dep IN LISTS LIB_MPU_DCACHE_DEPS)
    if(TARGET ${dep})
        target_link_libraries(lib_mpu_dcache PUBLIC ${dep})
    else()
        message(FATAL_ERROR
                "[lib_mpu_dcache] Required dependency target not found: ${dep}\n"
                " - Check Frameworks/${dep}/ builds a target named \"${dep}\"\n"
                " - Or update LIB_MPU_DCACHE_DEPS in ${CMAKE_CURRENT_LIST_FILE}")
    endif()
endforeach()

# 6) Export linker fragment path for top-level ordering (main script first)
set_property(TARGET lib_mpu_dcache PROPERTY OMNIX_DMA_NC_LD "${DMA_NC_LD_BIN}")

# 6.1) 让消费者目标在链接时感知到这个文件，改了会触发 relink
set_property(TARGET lib_mpu_dcache APPEND PROPERTY
        INTERFACE_LINK_DEPENDS "${DMA_NC_LD_BIN}"
)

# 6.2) ✅ 最关键：把“顺序守卫库”作为本模块的 PUBLIC 依赖
#      -> 任何可执行只要 link 到 lib_mpu_dcache，就会先构建 lib_mpu_dcache_order，
#         而它又依赖 lib_mpu_dcache_linker_fragment（复制出 dma_nc.ld）。
target_link_libraries(lib_mpu_dcache PUBLIC lib_mpu_dcache_order)

target_compile_definitions(lib_mpu_dcache
        INTERFACE OMNIX_MPU_DCACHE=1)

target_compile_definitions(mcu_core
        INTERFACE "$<$<TARGET_EXISTS:OmniX::lib_mpu_dcache>:OMNIX_FEATURE_MPU_DCACHE=1>")

# 7) 模块私有编译选项（可选）
# target_compile_definitions(lib_mpu_dcache PUBLIC LIB_MPU_DCACHE_ENABLE_LOG=1)
# target_compile_options(lib_mpu_dcache PRIVATE -Wall -Wextra)

# 8) 命名空间别名
add_library(OmniX::lib_mpu_dcache ALIAS lib_mpu_dcache)

# 9) 向上层导出目标名（顶层自动收集/链接）
set(FRAMEWORK_EXPORT_TARGET lib_mpu_dcache PARENT_SCOPE)
