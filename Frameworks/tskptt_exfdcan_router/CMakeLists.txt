# Frameworks/taskptt_exfdcan_router/CMakeLists.txt
cmake_minimum_required(VERSION 3.21)

set(TSKPTT_EXFDCAN_DEPS
        lib_mcp2518fd
        bsp_spi
        bsp_srn_log
)

set(TSKPTT_EXFDCAN_OPTIONAL_DEPS
        lib_adp_dji_c620
        lib_adp_dji_gm6020
        lib_adp_dm_4310
        lib_adp_dm_4340
        lib_adp_lk_mg8016e
        lib_adp_navision
        lib_imu
)

add_library(tskptt_exfdcan_router STATIC
        Inc/EXFDCAN_Router.h
        Src/tskptt_exfdcan_rx_router.cpp
        Src/tskptt_exfdcan_tx_router.cpp
        Src/tskptt_exfdcan_device.cpp
)

file(GLOB_RECURSE TSKPTT_EXFDCAN_SRC CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.S"
)
target_sources(tskptt_exfdcan_router PRIVATE ${TSKPTT_EXFDCAN_SRC})

target_include_directories(tskptt_exfdcan_router
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)

target_link_libraries(tskptt_exfdcan_router PUBLIC mcu_core)

foreach(dep IN LISTS TSKPTT_EXFDCAN_DEPS)
    if(TARGET ${dep})
        target_link_libraries(tskptt_exfdcan_router PUBLIC ${dep})
    else()
        message(FATAL_ERROR
                "[tskptt_exfdcan_router] Required dependency target not found: ${dep}\n"
                " - Check that Frameworks/${dep}/ builds a target named \"${dep}\"\n"
                " - Or update TSKPTT_EXFDCAN_DEPS in ${CMAKE_CURRENT_LIST_FILE}")
    endif()
endforeach()

foreach(dep IN LISTS TSKPTT_EXFDCAN_OPTIONAL_DEPS)
    if(TARGET ${dep})
        target_link_libraries(tskptt_exfdcan_router PUBLIC ${dep})
    else()
        message(STATUS
                "[tskptt_exfdcan_router] Optional dependency target not found: ${dep}")
    endif()
endforeach()

add_library(OmniX::tskptt_exfdcan_router ALIAS tskptt_exfdcan_router)
set(FRAMEWORK_EXPORT_TARGET tskptt_exfdcan_router PARENT_SCOPE)
