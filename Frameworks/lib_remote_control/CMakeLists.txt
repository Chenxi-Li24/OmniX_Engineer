# Frameworks/lib_remote_control/CMakeLists.txt
cmake_minimum_required(VERSION 3.21)

# 0) List dependencies (other framework libs). Edit to your actual needs.
#    Examples below: fdcan BSP, logging, and motor adaptors.
set(LIBRC_DEPS
        bsp_dr16
        bsp_srn_log
        bsp_vt03
)

# 1) Create library target (static)
add_library(lib_remote_control STATIC
        Inc/lib_remote_control.h)

# 2) Collect sources (auto refresh on changes)
file(GLOB_RECURSE LIB_REMOTE_CONTROL_SRC CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.S"
)
target_sources(lib_remote_control PRIVATE ${LIB_REMOTE_CONTROL_SRC})

# 3) Export public include dir
target_include_directories(lib_remote_control
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)

# 4) Inherit MCU flags/defines/includes from the shared interface (mcu_core)
#    This gives Cortex-M7 + FPU flags, HAL/FreeRTOS includes, etc.
target_link_libraries(lib_remote_control PUBLIC mcu_core)

# 5) Language/feature knobs (safe defaults; keep C/C++混合源码可编译)
#    如果纯C可移除cxx标准；如果需要，解开下面一行：
# target_compile_features(lib_remote_control PUBLIC cxx_std_17)

# 6) Link required dependent framework libs (transitively, with checks)
foreach(dep IN LISTS LIBRC_DEPS)
    if(TARGET ${dep})
        target_link_libraries(lib_remote_control PUBLIC ${dep})
    else()
        message(FATAL_ERROR
                "[lib_remote_control] Required dependency target not found: ${dep}\n"
                " - Check that Frameworks/${dep}/ builds a target named \"${dep}\"\n"
                " - Or update LIBRC_DEPS in ${CMAKE_CURRENT_LIST_FILE}")
    endif()
endforeach()

# 7) Optional: module-specific compile definitions / options (don’t leak globally)
# target_compile_definitions(lib_remote_control PUBLIC LIB_REMOTE_CONTROL_USE_TIM1)
# target_compile_options(lib_remote_control PRIVATE -Wall -Wextra)

# 8) Provide a namespaced ALIAS for convenience
add_library(OmniX::lib_remote_control ALIAS lib_remote_control)

# 9) Export target name upward so the top-level can auto-collect/link it
set(FRAMEWORK_EXPORT_TARGET lib_remote_control PARENT_SCOPE)