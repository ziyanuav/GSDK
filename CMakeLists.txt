cmake_minimum_required(VERSION 3.16)

project(gsdk_example VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# 输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# 目标平台推断（默认 x86）
if(NOT DEFINED TARGET_PLATFORM)
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
        set(TARGET_PLATFORM "aarch64")
    else()
        set(TARGET_PLATFORM "x86")
    endif()
endif()
message(STATUS "GSDK Example 目标平台: ${TARGET_PLATFORM}")

# 头文件目录（已在仓库准备好的对外 include）
set(GSDK_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)

include_directories(
    ${GSDK_INCLUDE_DIR}
    ${GSDK_INCLUDE_DIR}/3rd
    ${GSDK_INCLUDE_DIR}/framework
    ${GSDK_INCLUDE_DIR}/framework/common
    ${GSDK_INCLUDE_DIR}/framework/engine
    ${GSDK_INCLUDE_DIR}/framework/media
    ${GSDK_INCLUDE_DIR}/nlohmann
    ${GSDK_INCLUDE_DIR}/protocol
)

# 平台特定库目录（lib/x86 或 lib/aarch64）
set(GSDK_LIB_DIR ${CMAKE_SOURCE_DIR}/lib)
set(GSDK_LIB_PLATFORM_DIR ${GSDK_LIB_DIR}/${TARGET_PLATFORM})

# 库搜索路径 - 优先使用平台特定目录
link_directories(
    ${GSDK_LIB_PLATFORM_DIR}
)

# 绝对路径定位 Ziyan 主库
set(ZIYAN_ABS_LIB)
if(EXISTS ${GSDK_LIB_PLATFORM_DIR}/libZiyan.so)
    set(ZIYAN_ABS_LIB ${GSDK_LIB_PLATFORM_DIR}/libZiyan.so)
    message(STATUS "找到 Ziyan 库: ${ZIYAN_ABS_LIB}")
elseif(EXISTS ${GSDK_LIB_DIR}/libZiyan.so)
    set(ZIYAN_ABS_LIB ${GSDK_LIB_DIR}/libZiyan.so)
    message(STATUS "找到 Ziyan 库: ${ZIYAN_ABS_LIB}")
else()
    message(WARNING "未找到 libZiyan.so")
endif()

# 公共依赖库（与主工程保持一致的最小集合）
set(COMMON_LIBS
    ${ZIYAN_ABS_LIB}
    ssl crypto z curl zip tinyxml2 jaiotcppsdk pcre2-8 sqlite3
)

# 追加通过绝对路径的 OSG 与 ffmpeg 关键库（处理无无版本名的情况）
set(OSG_DIR ${GSDK_LIB_PLATFORM_DIR}/OSG365)
set(FFMPEG_DIR ${GSDK_LIB_PLATFORM_DIR}/ffmpeg)

if(EXISTS ${OSG_DIR})
    file(GLOB OSG_ABS_LIBS
        ${OSG_DIR}/libOpenThreads.so*
        ${OSG_DIR}/libosg.so*
        ${OSG_DIR}/libosgDB.so*
        ${OSG_DIR}/libosgAnimation.so*
        ${OSG_DIR}/libosgGA.so*
        ${OSG_DIR}/libosgViewer.so*
        ${OSG_DIR}/libosgUtil.so*
        ${OSG_DIR}/libosgText.so*
    )
    message(STATUS "找到 OSG 库: ${OSG_ABS_LIBS}")
endif()

if(EXISTS ${FFMPEG_DIR})
    file(GLOB SWRESAMPLE_ABS_LIB ${FFMPEG_DIR}/libswresample.so*)
    message(STATUS "找到 swresample 库: ${SWRESAMPLE_ABS_LIB}")
endif()

if(TARGET_PLATFORM STREQUAL "x86")
    set(PLATFORM_LIBS
        ${COMMON_LIBS}
        avcodec avdevice swscale avformat avutil
        ${SWRESAMPLE_ABS_LIB}
        ${OSG_ABS_LIBS}
        pthread
    )
elseif(TARGET_PLATFORM STREQUAL "aarch64")
    set(PLATFORM_LIBS
        ${COMMON_LIBS}
        avcodec avdevice swresample swscale avfilter avformat avutil
        pthread
    )
else()
    message(FATAL_ERROR "不支持的平台: ${TARGET_PLATFORM}")
endif()

# 示例源文件（可按需添加更多）
file(GLOB EXAMPLE_SOURCES
    ${CMAKE_SOURCE_DIR}/examples/*.cpp
)

add_executable(gsdk_basic_example ${EXAMPLE_SOURCES})

# 链接 Ziyan 以及平台库
target_link_libraries(gsdk_basic_example ${PLATFORM_LIBS})

# 运行时查找库路径（无需手动设置 LD_LIBRARY_PATH）
set_target_properties(gsdk_basic_example PROPERTIES
    BUILD_RPATH "${GSDK_LIB_PLATFORM_DIR};${GSDK_LIB_PLATFORM_DIR}/3rd;${GSDK_LIB_PLATFORM_DIR}/boost;${GSDK_LIB_PLATFORM_DIR}/ctb;${GSDK_LIB_PLATFORM_DIR}/event;${GSDK_LIB_PLATFORM_DIR}/pcre2;${GSDK_LIB_PLATFORM_DIR}/quic;${GSDK_LIB_PLATFORM_DIR}/ffmpeg;${GSDK_LIB_PLATFORM_DIR}/OSG365"
    INSTALL_RPATH "${GSDK_LIB_PLATFORM_DIR};${GSDK_LIB_PLATFORM_DIR}/3rd;${GSDK_LIB_PLATFORM_DIR}/boost;${GSDK_LIB_PLATFORM_DIR}/ctb;${GSDK_LIB_PLATFORM_DIR}/event;${GSDK_LIB_PLATFORM_DIR}/pcre2;${GSDK_LIB_PLATFORM_DIR}/quic;${GSDK_LIB_PLATFORM_DIR}/ffmpeg;${GSDK_LIB_PLATFORM_DIR}/OSG365"
)

# 复制配置（如需）
add_custom_target(copy_config ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_SOURCE_DIR}/config ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/config
    COMMENT "复制配置文件到输出目录"
)
add_dependencies(gsdk_basic_example copy_config)

message(STATUS "================ GSDK Example 配置 ================")
message(STATUS "包含目录: ${GSDK_INCLUDE_DIR}")
message(STATUS "库目录: ${GSDK_LIB_DIR}")
message(STATUS "平台库目录: ${GSDK_LIB_PLATFORM_DIR}")
message(STATUS "构建类型: ${CMAKE_BUILD_TYPE}")
message(STATUS "输出目录: ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
message(STATUS "==================================================")

