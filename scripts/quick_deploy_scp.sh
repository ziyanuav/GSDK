#!/usr/bin/env bash
# 快速部署：构建发布包并通过 SCP 上传到设备并安装
#
# 用法:
#   ./scripts/quick_deploy_scp.sh [设备IP/主机名] [版本号]
#
# 设备连接优先读取 deploy/device.conf（见 deploy/device.conf.example）
# 环境变量可覆盖: ZYGSDK_DEVICE_HOST / ZYGSDK_DEVICE_USER / ZYGSDK_DEVICE_PASSWORD 等
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck source=load_deploy_config.sh
source "${ROOT_DIR}/scripts/load_deploy_config.sh"

load_deploy_config "${ROOT_DIR}"

# 命令行参数覆盖配置文件
if [[ -n "${1:-}" ]]; then
  DEVICE_HOST="$1"
  export DEVICE_HOST
fi
VERSION="${2:-}"

require_device_host

echo "=========================================="
print_deploy_config
echo "=========================================="

cd "${ROOT_DIR}"
if [[ -n "${VERSION}" ]]; then
  "${ROOT_DIR}/scripts/build_zygsdk_release.sh" "${VERSION}"
else
  "${ROOT_DIR}/scripts/build_zygsdk_release.sh"
fi

if [[ -z "${VERSION}" ]]; then
  VERSION="$(tr -d ' \r\n' < "${ROOT_DIR}/VERSION")"
fi

TARBALL="${ROOT_DIR}/dist/zygsdk_${VERSION}_aarch64.tar.gz"
MD5_FILE="${TARBALL}.md5"
PKG_NAME="zygsdk_${VERSION}_aarch64"
REMOTE_TARBALL="$(deploy_remote_path "$(basename "${TARBALL}")")"
REMOTE_MD5="$(deploy_remote_path "$(basename "${MD5_FILE}")")"

if [[ ! -f "${TARBALL}" ]]; then
  echo "错误: 未找到 ${TARBALL}" >&2
  exit 1
fi

echo ""
echo "==> 上传到 $(deploy_target):${REMOTE_TARBALL}"
deploy_scp "${TARBALL}" "$(deploy_target):${REMOTE_TARBALL}"
if [[ -f "${MD5_FILE}" ]]; then
  echo "==> 上传 MD5: ${MD5_FILE}"
  deploy_scp "${MD5_FILE}" "$(deploy_target):${REMOTE_MD5}"
fi

echo ""
echo "==> 远程校验、解压并安装到 ${INSTALL_DIR} ..."
deploy_ssh bash -s <<EOF
set -e
cd "${REMOTE_TMP}"
if [[ -f $(basename "${MD5_FILE}") ]]; then
  echo "==> MD5 校验 ..."
  md5sum -c $(basename "${MD5_FILE}")
fi
rm -rf ${PKG_NAME}
tar xzf $(basename "${REMOTE_TARBALL}")
cd ${PKG_NAME}
chmod +x install.sh run.sh setup.sh bin/gsdk_basic_example
./install.sh install ${INSTALL_DIR}
EOF

echo ""
echo "==> 部署完成"
echo "  SSH:  ssh -p ${DEVICE_PORT} $(deploy_target)"
echo "  启动: ${INSTALL_DIR}/run.sh"
echo "  状态: ssh -p ${DEVICE_PORT} $(deploy_target) 'sudo ${INSTALL_DIR}/install.sh status ${INSTALL_DIR}'"
