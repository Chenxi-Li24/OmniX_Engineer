cmake_minimum_required(VERSION 3.21)

# 1) Create library target (static)
add_library(lib_adp_lk_mg8016e STATIC
        Inc/lib_adp_lk_mg8016e.h)

# 2) Collect sources (C/C++/ASM)
file(GLOB_RECURSE LIB_ADP_LK_MG8016E_SRC CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.S"
)
target_sources(lib_adp_lk_mg8016e PRIVATE ${LIB_ADP_LK_MG8016E_SRC})

# 3) Public include dir
target_include_directories(lib_adp_lk_mg8016e
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)

# 4) Inherit MCU flags/defines/includes
target_link_libraries(lib_adp_lk_mg8016e PUBLIC mcu_core)

# 5) (optional) module-specific options
# target_compile_definitions(lib_adp_lk_mg8016e PUBLIC LK_MG8016E_SOME_SWITCH=1)
# target_compile_options(lib_adp_lk_mg8016e PRIVATE -Wall -Wextra)

# 6) Export target name upward (top-level CMake will auto-link)
set(FRAMEWORK_EXPORT_TARGET lib_adp_lk_mg8016e PARENT_SCOPE)
