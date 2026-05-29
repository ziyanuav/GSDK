cmake_minimum_required(VERSION 3.16)

project(gsdk_example VERSION 1.0.1 LANGUAGES C CXX)

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

# 库搜索路径 - 与 BUILD_RPATH 子目录保持一致
link_directories(
    ${GSDK_LIB_PLATFORM_DIR}
    ${GSDK_LIB_PLATFORM_DIR}/3rd
    ${GSDK_LIB_PLATFORM_DIR}/boringssl
    ${GSDK_LIB_PLATFORM_DIR}/boost
    ${GSDK_LIB_PLATFORM_DIR}/ctb
    ${GSDK_LIB_PLATFORM_DIR}/event
    ${GSDK_LIB_PLATFORM_DIR}/event/lib
    ${GSDK_LIB_PLATFORM_DIR}/pcre2
    ${GSDK_LIB_PLATFORM_DIR}/quic
    ${GSDK_LIB_PLATFORM_DIR}/ffmpeg
    ${GSDK_LIB_PLATFORM_DIR}/OSG365
    ${GSDK_LIB_PLATFORM_DIR}/gdal
    ${GSDK_LIB_PLATFORM_DIR}/datachannel
    ${GSDK_LIB_PLATFORM_DIR}/datachannel/lib
    ${GSDK_LIB_PLATFORM_DIR}/mosquitto
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
if(TARGET_PLATFORM STREQUAL "aarch64")
    set(COMMON_LIBS
        ${ZIYAN_ABS_LIB}
        ${GSDK_LIB_PLATFORM_DIR}/libssl.so.3
        ${GSDK_LIB_PLATFORM_DIR}/libcrypto.so.3
        ${GSDK_LIB_PLATFORM_DIR}/libz.so.1
        ${GSDK_LIB_PLATFORM_DIR}/libcurl.so.4
        ${GSDK_LIB_PLATFORM_DIR}/libzip.so.5
        ${GSDK_LIB_PLATFORM_DIR}/libtinyxml2.so.11
        ${GSDK_LIB_PLATFORM_DIR}/libjaiotcppsdk.so
        ${GSDK_LIB_PLATFORM_DIR}/libpcre2-8.so.0
        ${GSDK_LIB_PLATFORM_DIR}/libsqlite3.so.0
    )
else()
    set(COMMON_LIBS
        ${ZIYAN_ABS_LIB}
        ssl crypto z curl zip tinyxml2 jaiotcppsdk pcre2-8 sqlite3
    )
endif()

# 第三方库目录
set(OSG_DIR ${GSDK_LIB_PLATFORM_DIR}/OSG365)
set(FFMPEG_DIR ${GSDK_LIB_PLATFORM_DIR}/ffmpeg)

# libZiyan.so 的传递依赖（部分第三方 .so 内置了错误的 RUNPATH）
# 通过 --no-as-needed 显式链接，由可执行文件的 RPATH 在启动时优先加载
set(PRELOAD_LIBS)
if(TARGET_PLATFORM STREQUAL "x86")
    if(EXISTS ${OSG_DIR})
        list(APPEND PRELOAD_LIBS
            ${OSG_DIR}/libOpenThreads.so.21
            ${OSG_DIR}/libosg.so.161
            ${OSG_DIR}/libosgDB.so.161
            ${OSG_DIR}/libosgAnimation.so.161
            ${OSG_DIR}/libosgGA.so.161
            ${OSG_DIR}/libosgViewer.so.161
            ${OSG_DIR}/libosgUtil.so.161
            ${OSG_DIR}/libosgText.so.161
        )
    endif()
    if(EXISTS ${FFMPEG_DIR})
        list(APPEND PRELOAD_LIBS
            ${FFMPEG_DIR}/libavcodec.so.61
            ${FFMPEG_DIR}/libswscale.so.8
            ${FFMPEG_DIR}/libavformat.so.61
            ${FFMPEG_DIR}/libavutil.so.59
            ${FFMPEG_DIR}/libswresample.so.5
        )
    endif()
    list(APPEND PRELOAD_LIBS
        ${GSDK_LIB_PLATFORM_DIR}/gdal/libgdal.so.36
        ${GSDK_LIB_PLATFORM_DIR}/ctb/libctb.so
        ${GSDK_LIB_PLATFORM_DIR}/event/libevent-2.1.so.7
        ${GSDK_LIB_PLATFORM_DIR}/boost/libboost_filesystem.so.1.86.0
        ${GSDK_LIB_PLATFORM_DIR}/boost/libboost_iostreams.so.1.86.0
        ${GSDK_LIB_PLATFORM_DIR}/datachannel/libdatachannel.so.0.21
        ${GSDK_LIB_PLATFORM_DIR}/mosquitto/libmosquitto.so.1
    )
elseif(TARGET_PLATFORM STREQUAL "aarch64")
    # ffmpeg 在交叉链接时直接预加载会触发大量 libc 符号检查，改由 RPATH 在目标机运行时加载
    list(APPEND PRELOAD_LIBS
        ${GSDK_LIB_PLATFORM_DIR}/event/lib/libevent-2.1.so.7
        ${GSDK_LIB_PLATFORM_DIR}/datachannel/lib/libdatachannel.so.0.21
        ${GSDK_LIB_PLATFORM_DIR}/boost/libboost_filesystem-mt-a64.so.1.86.0
        ${GSDK_LIB_PLATFORM_DIR}/boost/libboost_system-mt-a64.so.1.86.0
        ${GSDK_LIB_PLATFORM_DIR}/boost/libboost_iostreams.so.1.86.0
    )
endif()
if(PRELOAD_LIBS)
    message(STATUS "预加载传递依赖库: ${PRELOAD_LIBS}")
endif()

if(TARGET_PLATFORM STREQUAL "x86" OR TARGET_PLATFORM STREQUAL "aarch64")
    set(PLATFORM_LIBS
        -Wl,--no-as-needed
        ${PRELOAD_LIBS}
        -Wl,--as-needed
        ${COMMON_LIBS}
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

# 运行时库搜索路径（嵌入可执行文件，无需手动 export LD_LIBRARY_PATH）
set(GSDK_RUNTIME_RPATH
    "${GSDK_LIB_PLATFORM_DIR}"
    "${GSDK_LIB_PLATFORM_DIR}/3rd"
    "${GSDK_LIB_PLATFORM_DIR}/boringssl"
    "${GSDK_LIB_PLATFORM_DIR}/boost"
    "${GSDK_LIB_PLATFORM_DIR}/ctb"
    "${GSDK_LIB_PLATFORM_DIR}/event"
    "${GSDK_LIB_PLATFORM_DIR}/event/lib"
    "${GSDK_LIB_PLATFORM_DIR}/pcre2"
    "${GSDK_LIB_PLATFORM_DIR}/quic"
    "${GSDK_LIB_PLATFORM_DIR}/ffmpeg"
    "${GSDK_LIB_PLATFORM_DIR}/OSG365"
    "${GSDK_LIB_PLATFORM_DIR}/gdal"
    "${GSDK_LIB_PLATFORM_DIR}/datachannel"
    "${GSDK_LIB_PLATFORM_DIR}/datachannel/lib"
    "${GSDK_LIB_PLATFORM_DIR}/mosquitto"
)
string(JOIN ";" GSDK_RUNTIME_RPATH_STR ${GSDK_RUNTIME_RPATH})

# 交叉编译时让链接器解析传递依赖（libc、libz 等）
if(TARGET_PLATFORM STREQUAL "aarch64" AND CMAKE_CROSSCOMPILING)
    set(GSDK_RPATH_LINK_DIRS
        ${CMAKE_SYSROOT}/lib
        ${CMAKE_SYSROOT}/usr/lib
        ${GSDK_RUNTIME_RPATH}
    )
    foreach(_dir IN LISTS GSDK_RPATH_LINK_DIRS)
        set(_rpath_link_flags "${_rpath_link_flags} -Wl,-rpath-link,${_dir}")
    endforeach()
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${_rpath_link_flags}")
endif()

# 使用 RPATH（非 RUNPATH），以便传递依赖也能找到子目录中的 .so
set(_gsdk_link_flags "-Wl,--disable-new-dtags")
if(TARGET_PLATFORM STREQUAL "aarch64" AND CMAKE_CROSSCOMPILING)
    list(APPEND _gsdk_link_flags
        "-Wl,--allow-shlib-undefined"
        "-Wl,--unresolved-symbols=ignore-in-shared-libs"
    )
endif()
string(JOIN " " _gsdk_link_flags_str ${_gsdk_link_flags})
set_target_properties(gsdk_basic_example PROPERTIES
    BUILD_RPATH "${GSDK_RUNTIME_RPATH_STR}"
    INSTALL_RPATH "${GSDK_RUNTIME_RPATH_STR}"
    BUILD_WITH_INSTALL_RPATH TRUE
    LINK_FLAGS "${_gsdk_link_flags_str}"
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

