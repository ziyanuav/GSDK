#!/usr/bin/env bash
# 交叉编译 Linux glibc 版 ffmpeg，替换 lib/aarch64/ffmpeg 中的 Android 版
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
FFMPEG_SRC="${FFMPEG_SRC:-/home/liu/work/zy-cpp-gc/3rd/ffmpeg7.1.1}"
INSTALL_DIR="${ROOT_DIR}/lib/aarch64/ffmpeg"
STAGING="${ROOT_DIR}/lib/aarch64/.ffmpeg_build_staging"
TOOL_BIN="${ROOT_DIR}/Tool/bin"
SYSROOT="${ROOT_DIR}/Tool/aarch64-buildroot-linux-gnu/sysroot"
CROSS="aarch64-buildroot-linux-gnu-"

[[ -d "${FFMPEG_SRC}" ]] || { echo "错误: ffmpeg 源码不存在: ${FFMPEG_SRC}" >&2; exit 1; }
[[ -x "${TOOL_BIN}/${CROSS}gcc" ]] || { echo "错误: 工具链不存在" >&2; exit 1; }

if [[ -d "${INSTALL_DIR}" && ! -d "${INSTALL_DIR}.android.bak" ]]; then
  echo "==> 备份 Android ffmpeg -> ${INSTALL_DIR}.android.bak"
  cp -a "${INSTALL_DIR}" "${INSTALL_DIR}.android.bak"
fi

mkdir -p "${STAGING}"
cd "${FFMPEG_SRC}"

export PKG_CONFIG_PATH=""
export PKG_CONFIG_LIBDIR=""

./configure \
  --prefix="${STAGING}" \
  --libdir="${INSTALL_DIR}" \
  --enable-shared --disable-static --disable-doc --disable-programs \
  --arch=aarch64 --target-os=linux \
  --cross-prefix="${TOOL_BIN}/${CROSS}" \
  --sysroot="${SYSROOT}" --enable-cross-compile \
  --cc="${TOOL_BIN}/${CROSS}gcc" --cxx="${TOOL_BIN}/${CROSS}g++" \
  --strip="${TOOL_BIN}/${CROSS}strip" --enable-pic \
  --extra-cflags="-O2 -fPIC --sysroot=${SYSROOT}" \
  --extra-ldflags="-Wl,-rpath-link,${SYSROOT}/lib -Wl,-rpath-link,${SYSROOT}/usr/lib"

make -j"$(nproc)"
make install

if readelf -d "${INSTALL_DIR}/libavcodec.so"* 2>/dev/null | grep -q 'Shared library: \[libc\.so\]'; then
  echo "错误: 仍为 Android ffmpeg" >&2; exit 1
fi
echo "==> ffmpeg 已安装到 ${INSTALL_DIR}"
