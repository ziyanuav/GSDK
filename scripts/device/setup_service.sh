#!/usr/bin/env bash
# 在 NanoPC-T4 上注册 systemd 服务与 logrotate（一次性）
# 用法: sudo ./setup_service.sh [安装目录]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
INSTALL_DIR="${1:-/opt/zygsdk}"
SERVICE_NAME="${ZYGSDK_SERVICE_NAME:-zygsdk}"

if [[ "${EUID}" -ne 0 ]]; then
  echo "请使用 root 或 sudo 执行" >&2
  exit 1
fi

if [[ ! -f "${INSTALL_DIR}/run.sh" ]]; then
  echo "错误: ${INSTALL_DIR}/run.sh 不存在" >&2
  echo "" >&2
  echo "请先安装 GSDK（在当前解压目录执行）:" >&2
  echo "  cd /tmp/zygsdk_2.0.0.7_aarch64   # 版本号按实际" >&2
  echo "  chmod +x install.sh run.sh setup.sh bin/gsdk_basic_example" >&2
  echo "  sudo ./install.sh install ${INSTALL_DIR}" >&2
  echo "" >&2
  echo "或检查安装目录是否正确: ls -la ${INSTALL_DIR}/" >&2
  exit 1
fi

if [[ ! -x "${INSTALL_DIR}/run.sh" ]]; then
  echo "==> 修复 run.sh 可执行权限"
  chmod +x "${INSTALL_DIR}/run.sh" "${INSTALL_DIR}/setup.sh" 2>/dev/null || true
  chmod +x "${INSTALL_DIR}/bin/gsdk_basic_example" 2>/dev/null || true
fi

mkdir -p "${INSTALL_DIR}/bin/logs" "${INSTALL_DIR}/data/logs"

if [[ -f "${SCRIPT_DIR}/zygsdk.service" ]]; then
  sed "s|/opt/zygsdk|${INSTALL_DIR}|g" "${SCRIPT_DIR}/zygsdk.service" \
    > "/etc/systemd/system/${SERVICE_NAME}.service"
  systemctl daemon-reload
  echo "==> 已安装 systemd: /etc/systemd/system/${SERVICE_NAME}.service"
else
  echo "警告: 未找到 zygsdk.service 模板" >&2
fi

if [[ -f "${SCRIPT_DIR}/zygsdk.logrotate" ]]; then
  sed "s|/opt/zygsdk|${INSTALL_DIR}|g" "${SCRIPT_DIR}/zygsdk.logrotate" \
    > "/etc/logrotate.d/${SERVICE_NAME}"
  echo "==> 已安装 logrotate: /etc/logrotate.d/${SERVICE_NAME}"
else
  echo "警告: 未找到 zygsdk.logrotate 模板" >&2
fi

echo ""
echo "后续命令:"
echo "  systemctl start ${SERVICE_NAME}"
echo "  systemctl enable ${SERVICE_NAME}"
echo "  systemctl status ${SERVICE_NAME}"
