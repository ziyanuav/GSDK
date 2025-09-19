#!/usr/bin/env bash
set -euo pipefail

# 用法: ./scripts/build_gsdk_example.sh [x86|aarch64] [clean]
TARGET_PLATFORM="${1:-x86}"
DO_CLEAN="${2:-}"

if [[ "${TARGET_PLATFORM}" != "x86" && "${TARGET_PLATFORM}" != "aarch64" ]]; then
  echo "Usage: $0 [x86|aarch64] [clean]" >&2
  exit 1
fi

ROOT_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
BUILD_DIR="./build"
CMAKE_BIN="/usr/bin/cmake"
CMAKE_ROOT_DIR="/usr/share/cmake-3.22"

export CMAKE_ROOT="${CMAKE_ROOT_DIR}"

# 确保库目录存在
mkdir -p "./lib/${TARGET_PLATFORM}"

if [[ "${DO_CLEAN}" == "clean" ]]; then
  rm -rf "${BUILD_DIR}"
fi

# 配置与构建
if [[ "${TARGET_PLATFORM}" == "aarch64" ]]; then
  # 使用aarch64工具链
  "${CMAKE_BIN}" -S "./" -B "${BUILD_DIR}" \
    -DCMAKE_TOOLCHAIN_FILE="./cmake/toolchain-aarch64.cmake" \
    -DTARGET_PLATFORM="${TARGET_PLATFORM}"
else
  # 使用默认工具链
  "${CMAKE_BIN}" -S "./" -B "${BUILD_DIR}" -DTARGET_PLATFORM="${TARGET_PLATFORM}"
fi
"${CMAKE_BIN}" --build "${BUILD_DIR}" -j

# 运行提示
BIN_PATH="${BUILD_DIR}/bin/gsdk_basic_example"
echo "Build done. Binary: ${BIN_PATH}"
echo "If needed, set runtime path:"
echo "export LD_LIBRARY_PATH=$(pwd)/lib/${TARGET_PLATFORM}:$(pwd)/lib/${TARGET_PLATFORM}/3rd:$(pwd)/lib/${TARGET_PLATFORM}/boost:$(pwd)/lib/${TARGET_PLATFORM}/ctb:$(pwd)/lib/${TARGET_PLATFORM}/event:$(pwd)/lib/${TARGET_PLATFORM}/pcre2:$(pwd)/lib/${TARGET_PLATFORM}/OSG365:$(pwd)/lib/${TARGET_PLATFORM}/ffmpeg:\$LD_LIBRARY_PATH"
