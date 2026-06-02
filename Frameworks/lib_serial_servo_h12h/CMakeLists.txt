cmake_minimum_required(VERSION 3.21)

set(lib_serial_servo_h12h_DEPS
)

add_library(lib_serial_servo_h12h STATIC
        Inc/lib_serial_servo_h12h.h)

file(GLOB_RECURSE lib_serial_servo_h12h_SRC CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.S"
)
target_sources(lib_serial_servo_h12h PRIVATE ${lib_serial_servo_h12h_SRC})

target_include_directories(lib_serial_servo_h12h
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)

target_link_libraries(lib_serial_servo_h12h PUBLIC mcu_core)

foreach(dep IN LISTS lib_serial_servo_h12h_DEPS)
    if(TARGET ${dep})
        target_link_libraries(lib_serial_servo_h12h PUBLIC ${dep})
    else()
        message(FATAL_ERROR
                "[lib_serial_servo_h12h] Required dependency target not found: ${dep}\n"
                " - Check Frameworks/${dep}/ target name\n"
                " - Or update lib_serial_servo_h12h_DEPS in ${CMAKE_CURRENT_LIST_FILE}")
    endif()
endforeach()

add_library(OmniX::lib_serial_servo_h12h ALIAS lib_serial_servo_h12h)
set(FRAMEWORK_EXPORT_TARGET lib_serial_servo_h12h PARENT_SCOPE)


