cmake_minimum_required(VERSION 3.21)

add_library(lib_adp_dm_4340 STATIC
        Inc/lib_adp_dm_4340.h)

file(GLOB_RECURSE LIB_ADP_DM_4340_SRC CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.S"
)
target_sources(lib_adp_dm_4340 PRIVATE ${LIB_ADP_DM_4340_SRC})

target_include_directories(lib_adp_dm_4340
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)

target_link_libraries(lib_adp_dm_4340 PUBLIC mcu_core)

add_library(OmniX::lib_adp_dm_4340 ALIAS lib_adp_dm_4340)
set(FRAMEWORK_EXPORT_TARGET lib_adp_dm_4340 PARENT_SCOPE)

