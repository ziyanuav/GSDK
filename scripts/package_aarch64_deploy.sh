#!/usr/bin/env bash
set -euo pipefail

# 用法: ./scripts/package_aarch64_deploy.sh [输出目录]
# 默认输出到 D 盘 3588 部署目录

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
DEFAULT_OUT="/mnt/d/上云资料/GSDK资料/3588部署/gsdk"
OUT_DIR="${1:-$DEFAULT_OUT}"

BIN_SRC="${ROOT_DIR}/build/bin/gsdk_basic_example"
LIB_SRC="${ROOT_DIR}/lib/aarch64"
CONFIG_SRC="${ROOT_DIR}/config"

if [[ ! -f "${BIN_SRC}" ]]; then
  echo "错误: 未找到 aarch64 可执行文件，请先执行: ./scripts/build_gsdk_example.sh aarch64 clean" >&2
  exit 1
fi

if ! file "${BIN_SRC}" | grep -q "ARM aarch64"; then
  echo "错误: ${BIN_SRC} 不是 aarch64 可执行文件，请重新交叉编译" >&2
  exit 1
fi

PATCHELF="$(command -v patchelf || true)"
if [[ -z "${PATCHELF}" && -x "${ROOT_DIR}/Tool/bin/patchelf" ]]; then
  PATCHELF="${ROOT_DIR}/Tool/bin/patchelf"
fi

echo "==> 打包目录: ${OUT_DIR}"
rm -rf "${OUT_DIR}"
mkdir -p "${OUT_DIR}/bin" "${OUT_DIR}/lib"

echo "==> 复制可执行文件与配置"
cp -a "${BIN_SRC}" "${OUT_DIR}/bin/"
cp -a "${CONFIG_SRC}" "${OUT_DIR}/bin/"

echo "==> 复制运行时库 (lib/aarch64, 约 270MB)"
cp -a "${LIB_SRC}" "${OUT_DIR}/lib/"

# 将 RPATH 改为相对路径，便于在目标机任意目录部署
REL_RPATH='$ORIGIN/../lib/aarch64'
REL_RPATH+=':$ORIGIN/../lib/aarch64/3rd'
REL_RPATH+=':$ORIGIN/../lib/aarch64/boringssl'
REL_RPATH+=':$ORIGIN/../lib/aarch64/boost'
REL_RPATH+=':$ORIGIN/../lib/aarch64/ctb'
REL_RPATH+=':$ORIGIN/../lib/aarch64/event'
REL_RPATH+=':$ORIGIN/../lib/aarch64/event/lib'
REL_RPATH+=':$ORIGIN/../lib/aarch64/pcre2'
REL_RPATH+=':$ORIGIN/../lib/aarch64/quic'
REL_RPATH+=':$ORIGIN/../lib/aarch64/ffmpeg'
REL_RPATH+=':$ORIGIN/../lib/aarch64/OSG365'
REL_RPATH+=':$ORIGIN/../lib/aarch64/gdal'
REL_RPATH+=':$ORIGIN/../lib/aarch64/datachannel'
REL_RPATH+=':$ORIGIN/../lib/aarch64/datachannel/lib'
REL_RPATH+=':$ORIGIN/../lib/aarch64/mosquitto'

if [[ -n "${PATCHELF}" ]]; then
  echo "==> 修正 RPATH 为相对路径"
  "${PATCHELF}" --set-rpath "${REL_RPATH}" "${OUT_DIR}/bin/gsdk_basic_example"
else
  echo "警告: 未找到 patchelf，将生成 run.sh 作为备用启动脚本" >&2
fi

cat > "${OUT_DIR}/run.sh" <<'EOF'
#!/bin/sh
DIR="$(cd "$(dirname "$0")" && pwd)"
export LD_LIBRARY_PATH="${DIR}/lib/aarch64:${DIR}/lib/aarch64/3rd:${DIR}/lib/aarch64/boringssl:${DIR}/lib/aarch64/boost:${DIR}/lib/aarch64/ctb:${DIR}/lib/aarch64/event:${DIR}/lib/aarch64/event/lib:${DIR}/lib/aarch64/pcre2:${DIR}/lib/aarch64/quic:${DIR}/lib/aarch64/ffmpeg:${DIR}/lib/aarch64/OSG365:${DIR}/lib/aarch64/gdal:${DIR}/lib/aarch64/datachannel:${DIR}/lib/aarch64/datachannel/lib:${DIR}/lib/aarch64/mosquitto:${LD_LIBRARY_PATH:-}"
cd "${DIR}/bin"
exec ./gsdk_basic_example "$@"
EOF
chmod +x "${OUT_DIR}/run.sh"

cat > "${OUT_DIR}/README.txt" <<'EOF'
GSDK aarch64 部署包 (RK3588 等 ARM64 Linux)

目录结构:
  gsdk/
    run.sh                  启动脚本（推荐）
    bin/gsdk_basic_example  主程序
    bin/config/config.json  配置文件（按现场修改 IP/URL）
    lib/aarch64/            全部运行时依赖库

部署步骤:
  1. 将整个 gsdk 文件夹拷贝到 3588 设备（如 /opt/gsdk）
  2. chmod +x /opt/gsdk/run.sh /opt/gsdk/bin/gsdk_basic_example
  3. 编辑 bin/config/config.json 中的 drone.ip、fpv/gimbals 地址等
  4. 启动: /opt/gsdk/run.sh

或直接:
  cd /opt/gsdk/bin && ./gsdk_basic_example

运行后会在程序目录下自动创建 data/（日志、数据库）。

注意:
  - 需 ARM64 Linux（aarch64），不能在 x86 PC 上直接运行
  - 若提示缺库，用 run.sh 启动
EOF

echo "==> 完成"
du -sh "${OUT_DIR}"
echo "输出: ${OUT_DIR}"
echo "在 3588 上执行: ${OUT_DIR}/run.sh  (拷贝到设备后)"
