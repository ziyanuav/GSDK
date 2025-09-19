# AArch64交叉编译工具链文件
# 使用方法: cmake -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-aarch64.cmake ..

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# 设置工具链路径 - 指向工程根目录下的Tool文件夹
# 使用CMAKE_CURRENT_LIST_DIR获取工具链文件所在目录，然后推导出项目根目录
get_filename_component(PROJECT_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
set(TOOLCHAIN_PREFIX ${PROJECT_ROOT}/Tool)
set(TOOLCHAIN_BIN ${TOOLCHAIN_PREFIX}/bin)

# 检查工具链是否存在
if(NOT EXISTS ${TOOLCHAIN_BIN}/aarch64-buildroot-linux-gnu-gcc)
    message(FATAL_ERROR "AArch64工具链不存在: ${TOOLCHAIN_BIN}/aarch64-buildroot-linux-gnu-gcc")
endif()

# 设置编译器
set(CMAKE_C_COMPILER ${TOOLCHAIN_BIN}/aarch64-buildroot-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_BIN}/aarch64-buildroot-linux-gnu-g++)
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_BIN}/aarch64-buildroot-linux-gnu-gcc)

# 设置工具
set(CMAKE_AR ${TOOLCHAIN_BIN}/aarch64-buildroot-linux-gnu-ar)
set(CMAKE_RANLIB ${TOOLCHAIN_BIN}/aarch64-buildroot-linux-gnu-ranlib)
set(CMAKE_STRIP ${TOOLCHAIN_BIN}/aarch64-buildroot-linux-gnu-strip)
set(CMAKE_OBJCOPY ${TOOLCHAIN_BIN}/aarch64-buildroot-linux-gnu-objcopy)
set(CMAKE_OBJDUMP ${TOOLCHAIN_BIN}/aarch64-buildroot-linux-gnu-objdump)

# 设置系统根目录
set(CMAKE_SYSROOT ${TOOLCHAIN_PREFIX}/aarch64-buildroot-linux-gnu/sysroot)

# 设置查找路径
set(CMAKE_FIND_ROOT_PATH 
    ${CMAKE_SYSROOT}
    ${PROJECT_ROOT}/lib/aarch64
)

# 搜索程序在主机系统中查找
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# 库和头文件在目标系统中查找
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# 设置目标平台
set(TARGET_PLATFORM "aarch64")

# 编译器特定设置
set(CMAKE_C_FLAGS_INIT "-march=armv8-a")
set(CMAKE_CXX_FLAGS_INIT "-march=armv8-a")

message(STATUS "使用AArch64工具链: ${CMAKE_C_COMPILER}")
