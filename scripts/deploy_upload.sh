#!/usr/bin/env bash
# 仅上传已有 tar.gz 到设备（不重新编译）
# 用法: ./scripts/deploy_upload.sh [tar.gz路径]
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck source=load_deploy_config.sh
source "${ROOT_DIR}/scripts/load_deploy_config.sh"

load_deploy_config "${ROOT_DIR}"
require_device_host

TARBALL="${1:-}"
if [[ -z "${TARBALL}" ]]; then
  VERSION="$(tr -d ' \r\n' < "${ROOT_DIR}/VERSION")"
  TARBALL="${ROOT_DIR}/dist/zygsdk_${VERSION}_aarch64.tar.gz"
fi

if [[ ! -f "${TARBALL}" ]]; then
  echo "错误: 安装包不存在: ${TARBALL}" >&2
  exit 1
fi

REMOTE_TARBALL="$(deploy_remote_path "$(basename "${TARBALL}")")"

print_deploy_config
echo ""
echo "==> 上传 ${TARBALL}"
deploy_scp "${TARBALL}" "$(deploy_target):${REMOTE_TARBALL}"
echo "==> 已上传到 $(deploy_target):${REMOTE_TARBALL}"
