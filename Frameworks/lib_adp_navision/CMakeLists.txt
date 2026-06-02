# Frameworks/lib_adp_navision/CMakeLists.txt
cmake_minimum_required(VERSION 3.21)

# 1) Create library target (static)
add_library(lib_adp_navision STATIC
        Inc/lib_adp_navision.h)

# 2) Collect sources
file(GLOB_RECURSE LIB_ADP_NAVISION_SRC CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.S"
)
target_sources(lib_adp_navision PRIVATE ${LIB_ADP_NAVISION_SRC})

# 3) Public include dir = Inc/
target_include_directories(lib_adp_navision
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)

# 4) Inherit MCU flags/defines/includes from the shared interface (mcu_core)
target_link_libraries(lib_adp_navision PUBLIC mcu_core)

# 5) Export target name upward so the top-level can auto-collect/link it
set(FRAMEWORK_EXPORT_TARGET lib_adp_navision PARENT_SCOPE)
