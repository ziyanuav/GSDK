#!/usr/bin/env bash
# 紫燕 GSDK (aarch64) 安装 / 升级 / 回滚脚本（在设备上运行）
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DEFAULT_INSTALL_DIR="/opt/zygsdk"
BACKUP_ROOT="/opt/zygsdk_backup"

usage() {
  cat <<EOF
用法:
  sudo $0 install [安装目录]          首次安装或升级（默认 ${DEFAULT_INSTALL_DIR}）
  sudo $0 rollback [安装目录]         回滚到上一版本
  sudo $0 status [安装目录]           查看当前版本与备份
  sudo $0 uninstall [安装目录]        卸载（保留备份）

环境变量:
  ZYGSDK_KEEP_CONFIG=1   升级时保留已有 config.json（默认 1）
  ZYGSDK_KEEP_DATA=1     升级时保留 data 目录（默认 1）
EOF
}

read_version() {
  local dir="$1"
  if [[ -f "${dir}/VERSION" ]]; then
    tr -d ' \r\n' < "${dir}/VERSION"
  else
    echo "unknown"
  fi
}

ensure_root() {
  if [[ "${EUID}" -ne 0 ]]; then
    echo "请使用 root 或 sudo 执行" >&2
    exit 1
  fi
}

do_install() {
  local target="$1"
  local new_ver
  new_ver="$(read_version "${SCRIPT_DIR}")"
  local old_ver="none"
  local ts
  ts="$(date +%Y%m%d_%H%M%S)"

  if [[ -d "${target}" ]]; then
    old_ver="$(read_version "${target}")"
    mkdir -p "${BACKUP_ROOT}"
    local backup="${BACKUP_ROOT}/zygsdk_${old_ver}_${ts}"
    echo "==> 备份当前版本 ${old_ver} -> ${backup}"
    cp -a "${target}" "${backup}"
    echo "${backup}" > "${BACKUP_ROOT}/last_backup.path"
    echo "${old_ver}" > "${BACKUP_ROOT}/last_backup.version"

    if [[ "${ZYGSDK_KEEP_CONFIG:-1}" == "1" && -f "${target}/bin/config/config.json" ]]; then
      cp -a "${target}/bin/config/config.json" "/tmp/zygsdk_config.json.preserve"
    fi
    if [[ "${ZYGSDK_KEEP_DATA:-1}" == "1" && -d "${target}/data" ]]; then
      cp -a "${target}/data" "/tmp/zygsdk_data.preserve"
    fi
  fi

  echo "==> 安装 ${new_ver} 到 ${target}"
  mkdir -p "${target}"
  rsync -a --delete \
    "${SCRIPT_DIR}/bin" \
    "${SCRIPT_DIR}/lib" \
    "${SCRIPT_DIR}/run.sh" \
    "${SCRIPT_DIR}/setup.sh" \
    "${SCRIPT_DIR}/install.sh" \
    "${SCRIPT_DIR}/VERSION" \
    "${SCRIPT_DIR}/manifest.txt" \
    "${target}/" 2>/dev/null || {
      cp -a "${SCRIPT_DIR}/bin" "${SCRIPT_DIR}/lib" "${SCRIPT_DIR}/run.sh" \
            "${SCRIPT_DIR}/setup.sh" "${SCRIPT_DIR}/install.sh" \
            "${SCRIPT_DIR}/VERSION" "${SCRIPT_DIR}/manifest.txt" "${target}/"
    }

  if [[ -f "/tmp/zygsdk_config.json.preserve" ]]; then
    cp -a "/tmp/zygsdk_config.json.preserve" "${target}/bin/config/config.json"
    rm -f "/tmp/zygsdk_config.json.preserve"
    echo "==> 已保留原 config.json"
  fi
  if [[ -d "/tmp/zygsdk_data.preserve" ]]; then
    rm -rf "${target}/data"
    cp -a "/tmp/zygsdk_data.preserve" "${target}/data"
    rm -rf "/tmp/zygsdk_data.preserve"
    echo "==> 已保留原 data 目录"
  fi

  chmod +x "${target}/run.sh" "${target}/setup.sh" "${target}/install.sh" "${target}/bin/gsdk_basic_example" 2>/dev/null || true
  echo "${new_ver}" > "${target}/VERSION"
  echo "==> 安装完成: ${target} (版本 ${new_ver})"
  echo "    启动: ${target}/run.sh"
}

do_rollback() {
  local target="$1"
  if [[ ! -f "${BACKUP_ROOT}/last_backup.path" ]]; then
    echo "错误: 未找到可回滚的备份" >&2
    exit 1
  fi
  local backup
  backup="$(cat "${BACKUP_ROOT}/last_backup.path")"
  local old_ver
  old_ver="$(cat "${BACKUP_ROOT}/last_backup.version" 2>/dev/null || echo unknown)"
  if [[ ! -d "${backup}" ]]; then
    echo "错误: 备份目录不存在: ${backup}" >&2
    exit 1
  fi
  echo "==> 回滚 ${target} 到版本 ${old_ver}"
  rm -rf "${target}"
  cp -a "${backup}" "${target}"
  chmod +x "${target}/run.sh" "${target}/setup.sh" "${target}/install.sh" "${target}/bin/gsdk_basic_example" 2>/dev/null || true
  echo "==> 回滚完成"
}

do_status() {
  local target="$1"
  echo "安装目录: ${target}"
  if [[ -d "${target}" ]]; then
    echo "当前版本: $(read_version "${target}")"
  else
    echo "当前版本: 未安装"
  fi
  if [[ -f "${BACKUP_ROOT}/last_backup.path" ]]; then
    echo "最近备份: $(cat "${BACKUP_ROOT}/last_backup.path")"
    echo "备份版本: $(cat "${BACKUP_ROOT}/last_backup.version" 2>/dev/null || echo unknown)"
  fi
}

do_uninstall() {
  local target="$1"
  if [[ -d "${target}" ]]; then
    rm -rf "${target}"
    echo "==> 已卸载 ${target}（备份仍在 ${BACKUP_ROOT}）"
  fi
}

CMD="${1:-install}"
shift || true
TARGET="${1:-${DEFAULT_INSTALL_DIR}}"

case "${CMD}" in
  install)  ensure_root; do_install "${TARGET}" ;;
  rollback) ensure_root; do_rollback "${TARGET}" ;;
  status)   do_status "${TARGET}" ;;
  uninstall) ensure_root; do_uninstall "${TARGET}" ;;
  -h|--help|help) usage ;;
  *) usage; exit 1 ;;
esac
