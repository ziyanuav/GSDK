#!/usr/bin/env bash
# 紫燕 GSDK aarch64 发布包构建脚本
# 用法: ./scripts/build_zygsdk_release.sh [版本号] [输出目录]
# 示例: ./scripts/build_zygsdk_release.sh 2.0.0.7 ./dist
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "${ROOT_DIR}"

VERSION="${1:-}"
if [[ -z "${VERSION}" ]]; then
  if [[ -f "${ROOT_DIR}/VERSION" ]]; then
    VERSION="$(tr -d ' \r\n' < "${ROOT_DIR}/VERSION")"
  else
    VERSION="1.0.0"
  fi
fi

OUTPUT_BASE="${2:-${ROOT_DIR}/dist}"
STAGING="${OUTPUT_BASE}/.staging_zygsdk"
PKG_NAME="zygsdk_${VERSION}_aarch64"
PKG_DIR="${STAGING}/${PKG_NAME}"
TARBALL="${OUTPUT_BASE}/${PKG_NAME}.tar.gz"

PATCHELF="$(command -v patchelf || true)"
if [[ -z "${PATCHELF}" && -x "${ROOT_DIR}/Tool/bin/patchelf" ]]; then
  PATCHELF="${ROOT_DIR}/Tool/bin/patchelf"
fi

echo "=========================================="
echo " 紫燕 GSDK aarch64 发布包构建"
echo " 版本: ${VERSION}"
echo " 项目: ${ROOT_DIR}"
echo "=========================================="

echo ""
echo "[1/5] 交叉编译 aarch64 ..."
"${ROOT_DIR}/scripts/build_gsdk_example.sh" aarch64 clean

echo ""
echo "[2/5] 组装部署目录 ..."
rm -rf "${PKG_DIR}"
mkdir -p "${PKG_DIR}/bin" "${PKG_DIR}/lib"

cp -a "${ROOT_DIR}/build/bin/gsdk_basic_example" "${PKG_DIR}/bin/"
cp -a "${ROOT_DIR}/config" "${PKG_DIR}/bin/"

echo "    复制 lib/aarch64 (解引用符号链接，避免 tar/Windows 出现 0 字节) ..."
cp -RL "${ROOT_DIR}/lib/aarch64" "${PKG_DIR}/lib/"

echo "    校验 0 字节文件 ..."
ZERO_FILES="$(find "${PKG_DIR}/lib/aarch64" -type f -size 0 2>/dev/null | head -5 || true)"
if [[ -n "${ZERO_FILES}" ]]; then
  echo "错误: 发现 0 字节库文件:" >&2
  echo "${ZERO_FILES}" >&2
  exit 1
fi

echo "    校验 ffmpeg 必须为 Linux glibc 版 ..."
if find "${PKG_DIR}/lib/aarch64/ffmpeg" -type f \( -name '*.so' -o -name '*.so.*' \) -print0 2>/dev/null \
    | xargs -0 readelf -d 2>/dev/null | grep -q 'Shared library: \[libc\.so\]'; then
  echo "错误: ffmpeg 仍为 Android 版 (依赖 libc.so)" >&2
  echo "请先执行: ./scripts/build_ffmpeg_aarch64_linux.sh" >&2
  exit 1
fi

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
  echo "    设置相对 RPATH ..."
  "${PATCHELF}" --set-rpath "${REL_RPATH}" --force-rpath "${PKG_DIR}/bin/gsdk_basic_example"
fi

echo "${VERSION}" > "${PKG_DIR}/VERSION"
cat > "${PKG_DIR}/manifest.txt" <<EOF
package=${PKG_NAME}
version=${VERSION}
platform=aarch64
built_at=$(date -Iseconds)
built_on=$(uname -n)
git_commit=$(git rev-parse --short HEAD 2>/dev/null || echo unknown)
binary=bin/gsdk_basic_example
config=bin/config/config.json
EOF

cat > "${PKG_DIR}/run.sh" <<'EOF'
#!/bin/sh
DIR="$(cd "$(dirname "$0")" && pwd)"
export LD_LIBRARY_PATH="${DIR}/lib/aarch64:${DIR}/lib/aarch64/3rd:${DIR}/lib/aarch64/boringssl:${DIR}/lib/aarch64/boost:${DIR}/lib/aarch64/ctb:${DIR}/lib/aarch64/event:${DIR}/lib/aarch64/event/lib:${DIR}/lib/aarch64/pcre2:${DIR}/lib/aarch64/quic:${DIR}/lib/aarch64/ffmpeg:${DIR}/lib/aarch64/OSG365:${DIR}/lib/aarch64/gdal:${DIR}/lib/aarch64/datachannel:${DIR}/lib/aarch64/datachannel/lib:${DIR}/lib/aarch64/mosquitto:${LD_LIBRARY_PATH:-}"
cd "${DIR}/bin"
exec ./gsdk_basic_example "$@"
EOF

cat > "${PKG_DIR}/setup.sh" <<'EOF'
#!/bin/sh
DIR="$(cd "$(dirname "$0")" && pwd)"
chmod +x "${DIR}/run.sh" "${DIR}/setup.sh" "${DIR}/install.sh" "${DIR}/bin/gsdk_basic_example" 2>/dev/null || true
echo "权限已设置。启动: ${DIR}/run.sh"
EOF

cp "${ROOT_DIR}/scripts/device/install_zygsdk.sh" "${PKG_DIR}/install.sh"
chmod +x "${PKG_DIR}/run.sh" "${PKG_DIR}/setup.sh" "${PKG_DIR}/install.sh"

echo ""
echo "[3/5] 生成 tar.gz 安装包 ..."
mkdir -p "${OUTPUT_BASE}"
rm -f "${TARBALL}" "${TARBALL}.md5"
tar -C "${STAGING}" -czf "${TARBALL}" "${PKG_NAME}"
rm -rf "${STAGING}"

echo ""
echo "[4/5] 生成 MD5 校验文件 ..."
MD5_FILE="${TARBALL}.md5"
(
  cd "${OUTPUT_BASE}"
  md5sum "$(basename "${TARBALL}")" > "$(basename "${MD5_FILE}")"
)
PKG_MD5="$(awk '{print $1}' "${MD5_FILE}")"
echo "    MD5: ${PKG_MD5}"
echo "    文件: ${MD5_FILE}"

echo ""
echo "[5/5] 完成"
echo "  安装包: ${TARBALL}"
echo "  MD5:    ${MD5_FILE}"
echo "  大小:   $(du -h "${TARBALL}" | awk '{print $1}')"
echo ""
echo "设备解压前校验:"
echo "  cd /tmp && md5sum -c $(basename "${MD5_FILE}")"
echo ""
echo "部署到设备:"
if [[ -f "${ROOT_DIR}/deploy/device.conf" ]] || [[ -n "${ZYGSDK_DEVICE_HOST:-}" ]]; then
  # shellcheck source=load_deploy_config.sh
  source "${ROOT_DIR}/scripts/load_deploy_config.sh"
  load_deploy_config "${ROOT_DIR}" 2>/dev/null || true
  if [[ -n "${DEVICE_HOST:-}" ]]; then
    echo "  一键部署: ./scripts/quick_deploy_scp.sh"
    echo "  或手动:"
    echo "    scp ${TARBALL} ${TARBALL}.md5 ${DEVICE_USER:-root}@${DEVICE_HOST}:${REMOTE_TMP:-/tmp}/"
    echo "    ssh ${DEVICE_USER:-root}@${DEVICE_HOST}"
    echo "    cd ${REMOTE_TMP:-/tmp} && md5sum -c $(basename "${TARBALL}").md5 && tar xzf $(basename "${TARBALL}") && cd ${PKG_NAME} && sudo ./install.sh install ${INSTALL_DIR:-/opt/zygsdk}"
  else
    echo "  1) cp deploy/device.conf.example deploy/device.conf  # 配置 DEVICE_HOST"
    echo "  2) ./scripts/quick_deploy_scp.sh"
  fi
else
  echo "  1) cp deploy/device.conf.example deploy/device.conf  # 配置 IP/账号/密码"
  echo "  2) ./scripts/quick_deploy_scp.sh"
  echo "  或手动: scp ${TARBALL} <user>@<host>:/tmp/ && ssh <user>@<host> 解压安装"
fi
