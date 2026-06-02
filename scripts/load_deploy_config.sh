#!/usr/bin/env bash
# 加载 NanoPC-T4 等设备部署配置
# 优先级: 命令行/环境变量 > deploy/device.conf > deploy/device.conf.example
#
# 用法（在其他脚本中）:
#   source "$(dirname "$0")/load_deploy_config.sh"
#   load_deploy_config
#   deploy_scp file remote_path
#   deploy_ssh "command"

_load_deploy_config_dir() {
  local script_dir
  script_dir="$(cd "$(dirname "${BASH_SOURCE[1]:-${BASH_SOURCE[0]}}")" && pwd)"
  echo "$(cd "${script_dir}/.." && pwd)"
}

_find_deploy_conf() {
  local root="$1"
  local candidates=(
    "${ZYGSDK_DEPLOY_CONF:-}"
    "${root}/deploy/device.conf"
    "${HOME}/.config/zygsdk/device.conf"
    "${root}/deploy/device.conf.example"
  )
  local c
  for c in "${candidates[@]}"; do
    [[ -n "${c}" && -f "${c}" ]] || continue
    echo "${c}"
    return 0
  done
  return 1
}

load_deploy_config() {
  local root="${1:-$(_load_deploy_config_dir)}"
  local conf=""

  if conf="$(_find_deploy_conf "${root}")"; then
    # shellcheck disable=SC1090
    source "${conf}"
  fi

  # 环境变量覆盖配置文件
  DEVICE_HOST="${ZYGSDK_DEVICE_HOST:-${DEVICE_HOST:-}}"
  DEVICE_USER="${ZYGSDK_DEVICE_USER:-${DEVICE_USER:-root}}"
  DEVICE_PORT="${ZYGSDK_DEVICE_PORT:-${DEVICE_PORT:-22}}"
  DEVICE_PASSWORD="${ZYGSDK_DEVICE_PASSWORD:-${DEVICE_PASSWORD:-}}"
  SSH_KEY="${ZYGSDK_SSH_KEY:-${SSH_KEY:-}}"
  SSH_OPTS="${ZYGSDK_SSH_OPTS:-${SSH_OPTS:-}}"
  REMOTE_TMP="${ZYGSDK_REMOTE_TMP:-${REMOTE_TMP:-/tmp}}"
  INSTALL_DIR="${ZYGSDK_INSTALL_DIR:-${INSTALL_DIR:-/opt/zygsdk}}"

  export DEVICE_HOST DEVICE_USER DEVICE_PORT DEVICE_PASSWORD
  export SSH_KEY SSH_OPTS REMOTE_TMP INSTALL_DIR
}

require_device_host() {
  if [[ -z "${DEVICE_HOST:-}" ]]; then
    cat >&2 <<EOF
错误: 未配置设备地址 DEVICE_HOST

请任选一种方式:
  1) cp deploy/device.conf.example deploy/device.conf  并编辑 DEVICE_HOST
  2) export ZYGSDK_DEVICE_HOST=192.168.x.x
  3) 命令行传参: ./scripts/quick_deploy_scp.sh 192.168.x.x

EOF
    exit 1
  fi
}

_deploy_ssh_base_args() {
  _DEPLOY_SSH_ARGS=(-p "${DEVICE_PORT}" -o "ConnectTimeout=15")
  if [[ -n "${SSH_KEY:-}" ]]; then
    _DEPLOY_SSH_ARGS+=(-i "${SSH_KEY}")
  fi
  if [[ -n "${SSH_OPTS:-}" ]]; then
    # shellcheck disable=SC2206
    local extra=(${SSH_OPTS})
    _DEPLOY_SSH_ARGS+=("${extra[@]}")
  fi
}

_deploy_scp_base_args() {
  _DEPLOY_SCP_ARGS=(-P "${DEVICE_PORT}" -o "ConnectTimeout=15")
  if [[ -n "${SSH_KEY:-}" ]]; then
    _DEPLOY_SCP_ARGS+=(-i "${SSH_KEY}")
  fi
  if [[ -n "${SSH_OPTS:-}" ]]; then
    # shellcheck disable=SC2206
    local extra=(${SSH_OPTS})
    for opt in "${extra[@]}"; do
      _DEPLOY_SCP_ARGS+=(-o "${opt}")
    done
  fi
}

_deploy_with_sshpass() {
  if [[ -n "${DEVICE_PASSWORD:-}" ]] && command -v sshpass >/dev/null 2>&1; then
    SSHPASS=(sshpass -p "${DEVICE_PASSWORD}")
    return 0
  fi
  if [[ -n "${DEVICE_PASSWORD:-}" ]]; then
    echo "提示: 已配置 DEVICE_PASSWORD 但未安装 sshpass，将尝试交互输入密码" >&2
    echo "      安装: sudo apt install sshpass   或配置 SSH 密钥免密登录" >&2
  fi
  SSHPASS=()
}

deploy_ssh() {
  _deploy_ssh_base_args
  _deploy_with_sshpass
  "${SSHPASS[@]}" ssh "${_DEPLOY_SSH_ARGS[@]}" "${DEVICE_USER}@${DEVICE_HOST}" "$@"
}

deploy_scp() {
  _deploy_scp_base_args
  _deploy_with_sshpass
  "${SSHPASS[@]}" scp "${_DEPLOY_SCP_ARGS[@]}" "$@"
}

deploy_target() {
  echo "${DEVICE_USER}@${DEVICE_HOST}"
}

deploy_remote_path() {
  local name="$1"
  echo "${REMOTE_TMP%/}/${name}"
}

print_deploy_config() {
  echo "设备配置:"
  echo "  配置文件: ${ZYGSDK_DEPLOY_CONF:-${_last_conf:-自动检测}}"
  echo "  目标:     ${DEVICE_USER}@${DEVICE_HOST}:${DEVICE_PORT}"
  echo "  远程目录: ${REMOTE_TMP}"
  echo "  安装路径: ${INSTALL_DIR}"
  if [[ -n "${SSH_KEY:-}" ]]; then
    echo "  SSH 密钥: ${SSH_KEY}"
  fi
  if [[ -n "${DEVICE_PASSWORD:-}" ]]; then
    echo "  密码:     已配置（不会显示）"
  else
    echo "  密码:     未配置（使用密钥或交互输入）"
  fi
}
