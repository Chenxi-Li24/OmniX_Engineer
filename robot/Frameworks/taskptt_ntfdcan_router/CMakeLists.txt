# Frameworks/tskptt_ntfdcan_router/CMakeLists.txt
cmake_minimum_required(VERSION 3.21)

# 0) List dependencies (other framework libs). Edit to your actual needs.
#    Required: core CAN/BSP drivers. Optional: adapters and add-ons.
set(TSKPTT_REQUIRED_DEPS
        bsp_fdcan
        bsp_srn_log
)

set(TSKPTT_OPTIONAL_DEPS
        lib_adp_dji_gm6020
        lib_adp_dji_c620
        lib_adp_dm_4310
        lib_adp_dm_4340
        lib_adp_lk_mg8016e
        lib_adp_navision
        lib_imu
)

# 1) Create library target (static)
add_library(tskptt_ntfdcan_router STATIC
        Inc/NTFDCAN_Router.h
        Src/tskptt_ntfdcan_tx_router.cpp
)

# 2) Collect sources (auto refresh on changes)
file(GLOB_RECURSE TSKPTT_NTFDCAN_ROUTER_SRC CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.S"
)
target_sources(tskptt_ntfdcan_router PRIVATE ${TSKPTT_NTFDCAN_ROUTER_SRC})

# 3) Export public include dir
target_include_directories(tskptt_ntfdcan_router
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)

# 4) Inherit MCU flags/defines/includes from the shared interface (mcu_core)
#    This gives Cortex-M7 + FPU flags, HAL/FreeRTOS includes, etc.
target_link_libraries(tskptt_ntfdcan_router PUBLIC mcu_core)

# 5) Language/feature knobs (safe defaults; keep C/C++混合源码可编译)
#    如果纯C可移除cxx标准；如果需要，解开下面一行：
# target_compile_features(tskptt_ntfdcan_router PUBLIC cxx_std_17)

# 6) Link required dependent framework libs (fatal if missing)
foreach(dep IN LISTS TSKPTT_REQUIRED_DEPS)
    if(TARGET ${dep})
        target_link_libraries(tskptt_ntfdcan_router PUBLIC ${dep})
    else()
        message(FATAL_ERROR
                "[tskptt_ntfdcan_router] Required dependency target not found: ${dep}\n"
                " - Check that Frameworks/${dep}/ builds a target named \"${dep}\"\n"
                " - Or update TSKPTT_REQUIRED_DEPS in ${CMAKE_CURRENT_LIST_FILE}")
    endif()
endforeach()

# 7) Link optional dependent framework libs (only if present)
foreach(dep IN LISTS TSKPTT_OPTIONAL_DEPS)
    if(TARGET ${dep})
        target_link_libraries(tskptt_ntfdcan_router PUBLIC ${dep})
    else()
        message(STATUS
                "[tskptt_ntfdcan_router] Optional dependency target not found: ${dep}")
    endif()
endforeach()

# 8) Optional: module-specific compile definitions / options (don’t leak globally)
# target_compile_definitions(tskptt_ntfdcan_router PUBLIC TSKPTT_NTFDCAN_ROUTER_USE_TIM1)
# target_compile_options(tskptt_ntfdcan_router PRIVATE -Wall -Wextra)

# 9) Provide a namespaced ALIAS for convenience
add_library(OmniX::tskptt_ntfdcan_router ALIAS tskptt_ntfdcan_router)

# 10) Export target name upward so the top-level can auto-collect/link it
set(FRAMEWORK_EXPORT_TARGET tskptt_ntfdcan_router PARENT_SCOPE)
