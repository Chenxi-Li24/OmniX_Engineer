# THIS FILE IS AUTO GENERATED FROM THE TEMPLATE! DO NOT CHANGE!
# CMakeLists.txt for STM32H7xx Cortex-M7 projects using arm-none-eabi toolchain
# Auto-discovers Frameworks modules in the Frameworks/ folder
# by Sirin Ver 2.2 - 2026-01-19
# ==============================
# Toolchain & project basics
# ==============================
cmake_minimum_required(VERSION 3.21)
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)

# Cross toolchain
set(CMAKE_C_COMPILER   arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_AR           arm-none-eabi-ar)
set(CMAKE_OBJCOPY      arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP      arm-none-eabi-objdump)
set(SIZE               arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(h723vg-v2-freertos-engineer C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# ==============================
# MCU global config as INTERFACE
# ==============================
add_library(mcu_core INTERFACE)

# --- CPU/FPU/ABI flags ---
target_compile_options(mcu_core INTERFACE
        -mcpu=cortex-m7 -mthumb -mthumb-interwork
        -mfloat-abi=hard -mfpu=fpv5-sp-d16
        -ffunction-sections -fdata-sections -fno-common -fmessage-length=0
        $<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>
)
target_link_options(mcu_core INTERFACE
        -mcpu=cortex-m7 -mthumb -mthumb-interwork
        -mfloat-abi=hard -mfpu=fpv5-sp-d16
        -Wl,--gc-sections
)

# --- Optimize by build type ---
if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    target_compile_options(mcu_core INTERFACE -Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    target_compile_options(mcu_core INTERFACE -Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    target_compile_options(mcu_core INTERFACE -Os)
else()
    target_compile_options(mcu_core INTERFACE -Og -g)
endif()

# --- Common defines ---
target_compile_definitions(mcu_core INTERFACE
        DEBUG
        USE_PWR_LDO_SUPPLY
        USE_HAL_DRIVER
        STM32H723xx
        ARM_MATH_CM7
        ARM_MATH_MATRIX_CHECK
        ARM_MATH_ROUNDING
)

# ==============================
# Includes (keep your original CM4 path as requested)
# ==============================
set(FREERTOS_ROOT ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source)
set(FREERTOS_PORT_DIR ${FREERTOS_ROOT}/portable/GCC/ARM_CM4F)
if (NOT EXISTS "${FREERTOS_PORT_DIR}/portmacro.h")
    message(FATAL_ERROR "FreeRTOS ARM_CM4F port not found at: ${FREERTOS_PORT_DIR}")
endif()

target_include_directories(mcu_core INTERFACE
        ${CMAKE_SOURCE_DIR}/Core/Inc
        ${CMAKE_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc
        ${CMAKE_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32H7xx/Include
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
        ${FREERTOS_ROOT}/include
        ${FREERTOS_ROOT}/CMSIS_RTOS_V2
        ${FREERTOS_PORT_DIR}
        ${CMAKE_SOURCE_DIR}/Tasks/Inc
        ${CMAKE_SOURCE_DIR}/Applications/Inc
        ${CMAKE_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Inc
)

# ==============================
# Linker script & options
# ==============================
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32H723VGHX_FLASH.ld)
add_link_options(-Wl,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
target_link_options(mcu_core INTERFACE "LINKER:-T${LINKER_SCRIPT}")

# ==============================
# Auto-discover Frameworks modules
# ==============================
set(FRAMEWORKS_DIR "${CMAKE_SOURCE_DIR}/Frameworks")
file(GLOB FRAMEWORK_DIRS LIST_DIRECTORIES TRUE "${FRAMEWORKS_DIR}/*")

set(AUTOFW_TARGETS)  # collected module libraries

# --------------------------------
# 1) Priority load some frameworks
#    These modules must be available before others
# --------------------------------
set(FRAMEWORK_PRIORITY
        lib_mpu_dcache # MPU/DCACHE config helper
        bsp_srn_log    # logging framework used by many modules
        bsp_i2c        # 24c02 and others depend on this
        bsp_spi        # bmi088 and others depend on this
        lib_algos      # math algorithms used by many modules
        bsp_bmi088     # BMI088 IMU
        lib_imu        # IMU
)

foreach(mod IN LISTS FRAMEWORK_PRIORITY)
    set(dir "${FRAMEWORKS_DIR}/${mod}")
    if (EXISTS "${dir}/CMakeLists.txt")
        message(STATUS "[OMX Framework] add_subdirectory: ${mod} (priority)")
        add_subdirectory("${dir}")

        # collect exported target (same logic as normal path)
        if (DEFINED FRAMEWORK_EXPORT_TARGET)
            list(APPEND AUTOFW_TARGETS ${FRAMEWORK_EXPORT_TARGET})
            unset(FRAMEWORK_EXPORT_TARGET)
        elseif (TARGET ${mod})
            list(APPEND AUTOFW_TARGETS ${mod})
        endif()

        # remove from FRAMEWORK_DIRS to avoid adding it twice
        list(REMOVE_ITEM FRAMEWORK_DIRS "${dir}")
    endif()
endforeach()

# --------------------------------
# 2) Normal auto-discover for the rest
# --------------------------------
foreach(dir ${FRAMEWORK_DIRS})
    if (NOT IS_DIRECTORY "${dir}")
        continue()
    endif()
    get_filename_component(mod "${dir}" NAME)
    if (mod STREQUAL ".git" OR mod STREQUAL "build")
        continue()
    endif()

    if (EXISTS "${dir}/CMakeLists.txt")
        message(STATUS "[OMX Framework] add_subdirectory: ${mod}")
        add_subdirectory("${dir}")

        # === collect exported target here and clear it ===
        if (DEFINED FRAMEWORK_EXPORT_TARGET)
            list(APPEND AUTOFW_TARGETS ${FRAMEWORK_EXPORT_TARGET})
            unset(FRAMEWORK_EXPORT_TARGET)  # avoid leaking to next subdir
        elseif (TARGET ${mod})
            # fallback if target name equals folder name
            list(APPEND AUTOFW_TARGETS ${mod})
        endif()

    else()
        # Auto-pack Inc/Src into a static lib named <mod>
        set(INC_DIR "${dir}/Inc")
        set(SRC_DIR "${dir}/Src")
        file(GLOB_RECURSE MOD_SOURCES CONFIGURE_DEPENDS
                "${SRC_DIR}/*.c" "${SRC_DIR}/*.cpp" "${SRC_DIR}/*.cc"
                "${SRC_DIR}/*.S" "${SRC_DIR}/*.s"
        )
        if (MOD_SOURCES)
            set(t "${mod}")
            add_library(${t} STATIC ${MOD_SOURCES})
            target_link_libraries(${t} PUBLIC mcu_core)
            if (EXISTS "${INC_DIR}")
                target_include_directories(${t} PUBLIC "${INC_DIR}")
            endif()
            message(STATUS "[OMX Framework] auto STATIC: ${t}")
            list(APPEND AUTOFW_TARGETS ${t})
        endif()
    endif()
endforeach()

# Debug (optional):
# message(STATUS "AUTOFW_TARGETS = ${AUTOFW_TARGETS}")

# ==============================
# Optional linker fragment (keep main script first)
# ==============================
if (TARGET lib_mpu_dcache)
    get_target_property(MPU_DMA_NC_LD lib_mpu_dcache OMNIX_DMA_NC_LD)
    if (MPU_DMA_NC_LD AND NOT MPU_DMA_NC_LD STREQUAL "MPU_DMA_NC_LD-NOTFOUND")
        target_link_options(mcu_core INTERFACE "LINKER:-T${MPU_DMA_NC_LD}")
    endif()
endif()

# ==============================
# Main sources (exclude Frameworks/*)
# ==============================
file(GLOB_RECURSE APP_SOURCES CONFIGURE_DEPENDS
        "${CMAKE_SOURCE_DIR}/Core/*.*"
        "${CMAKE_SOURCE_DIR}/Middlewares/*.*"
        "${CMAKE_SOURCE_DIR}/Drivers/*.*"
        "${CMAKE_SOURCE_DIR}/Applications/*.*"
        "${CMAKE_SOURCE_DIR}/Tasks/*.*"
)
# Exclude non-source files and any Frameworks content (modules are separate libs)
list(FILTER APP_SOURCES EXCLUDE REGEX "\\.(ld|txt)$")
list(FILTER APP_SOURCES EXCLUDE REGEX "/Frameworks/")

add_executable(${PROJECT_NAME}.elf ${APP_SOURCES})
target_link_libraries(${PROJECT_NAME}.elf PRIVATE mcu_core ${AUTOFW_TARGETS})

# ==============================
# HEX/BIN artifacts
# ==============================
set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex   $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}\nBuilding ${BIN_FILE}"
)
