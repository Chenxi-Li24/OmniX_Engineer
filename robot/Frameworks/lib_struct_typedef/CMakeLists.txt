
project(struct_typedef C)

# 仅头文件，没有源文件，因此不用 add_library()
# 虽然没有 add_library，但为了兼容 add_subdirectory 可以注册一个 INTERFACE 库
add_library(struct_typedef INTERFACE
        Inc/struct_typedef.h)

# 让主项目通过 add_subdirectory 后自动继承 include path
target_include_directories(struct_typedef INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)