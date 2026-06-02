# 紫燕 GSDK — NanoPC-T4 安装验证与操作记录

本文档记录 **NanoPC-T4 设备安装后的验证步骤、预期输出及常见问题**，供现场运维与后续部署参考。

> 关联文档：[部署与升级指南](zygsdk-aarch64-deploy-upgrade.md)

---

## 一、验证环境（实测记录）

| 项目 | 实测值 |
|------|--------|
| 设备 | NanoPC-T4（RK3399，aarch64） |
| 操作系统 | Ubuntu **20.04.6** LTS（文档亦适用 22.04） |
| 内核 | 4.19.193 aarch64 |
| SSH | `ssh root@192.168.1.140` |
| 安装路径 | `/opt/zygsdk` |
| 安装版本 | `2.0.0.7` |
| 部署方式 | WSL 开发机 `./scripts/quick_deploy_scp.sh` |

---

## 二、登录设备

**Windows PowerShell：**

```powershell
ssh root@192.168.1.140
```

**预期输出（节选）：**

```text
Welcome to Ubuntu 20.04.6 LTS (GNU/Linux 4.19.193 aarch64)
root@NanoPC-T4:~#
```

---

## 三、安装目录验证

### 3.1 查看安装目录

```bash
ls -la /opt/zygsdk/
cat /opt/zygsdk/VERSION
```

**实测输出：**

```text
total 32
drwxr-xr-x 4 root root 4096 Dec 25 07:03 .
drwxr-xr-x 1 root root 4096 Dec 25 07:03 ..
-rw-r--r-- 1 pi   pi      8 Dec 25 07:03 VERSION
drwxr-xr-x 3 pi   pi   4096 Jun  2  2026 bin
drwxr-xr-x 3 pi   pi   4096 Jun  2  2026 lib
-rw-r--r-- 1 pi   pi    203 Jun  2  2026 manifest.txt
-rwxr-xr-x 1 pi   pi    533 Jun  2  2026 run.sh
-rwxr-xr-x 1 pi   pi    209 Jun  2  2026 setup.sh

2.0.0.7
```

**判定：** ✅ 安装成功，版本与目录结构正确。

**说明：**

- 文件属主可能为 `pi`，root 运行一般无问题。
- 目录时间显示 `2026` 为开发机与设备**时钟不同步**导致，不影响使用（见第六节）。

---

### 3.2 目录结构说明

```text
/opt/zygsdk/
├── VERSION              # 版本号
├── manifest.txt         # 构建信息
├── run.sh               # 启动脚本（必须使用）
├── setup.sh             # 修复可执行权限
├── install.sh           # 安装/升级/回滚（旧版安装包可能缺失，见 5.1）
├── bin/
│   ├── gsdk_basic_example
│   └── config/config.json
└── lib/aarch64/         # 运行时库
```

---

## 四、二进制与依赖验证

### 4.1 架构检查

```bash
file /opt/zygsdk/bin/gsdk_basic_example
```

**实测输出：**

```text
/opt/zygsdk/bin/gsdk_basic_example: ELF 64-bit LSB executable, ARM aarch64, version 1 (GNU/Linux), dynamically linked, interpreter /lib/ld-linux-aarch64.so.1, for GNU/Linux 4.19.0, not stripped
```

**判定：** ✅ 为 aarch64 可执行文件，可在 NanoPC-T4 运行。

---

### 4.2 动态库检查

```bash
ldd /opt/zygsdk/bin/gsdk_basic_example | grep "not found"
```

**实测输出：**（无输出）

**判定：** ✅ 所有依赖库均可解析，无缺失。

---

### 4.3 ffmpeg 库检查（可选）

```bash
readelf -d /opt/zygsdk/lib/aarch64/ffmpeg/libavcodec.so | grep NEEDED | head -5
```

**预期：** 应出现 `libc.so.6`，**不应**出现 Android 版 `libc.so`。

---

## 五、安装管理脚本

### 5.1 install.sh 缺失（已知问题，已修复）

**现象：**

```bash
/opt/zygsdk/install.sh status /opt/zygsdk
# -bash: /opt/zygsdk/install.sh: No such file or directory
```

**原因：** 早期 `install.sh` 安装时未复制到 `/opt/zygsdk`（仓库已修复，下次部署会自动带上）。

**不影响：** 程序启动与运行。

**手动补装：**

```bash
# 若 /tmp 解压目录仍在
cp /tmp/zygsdk_2.0.0.7_aarch64/install.sh /opt/zygsdk/
chmod +x /opt/zygsdk/install.sh
/opt/zygsdk/install.sh status /opt/zygsdk
```

**从开发机上传：**

```bash
# WSL
scp /path/to/GSDK/scripts/device/install_zygsdk.sh root@192.168.1.140:/opt/zygsdk/install.sh
ssh root@192.168.1.140 'chmod +x /opt/zygsdk/install.sh'
```

**status 预期输出（示例）：**

```text
安装目录: /opt/zygsdk
当前版本: 2.0.0.7
```

---

## 六、部署过程常见提示

### 6.0 MD5 校验

构建后会生成 `zygsdk_x.x.x.x_aarch64.tar.gz.md5`。解压前在设备上执行：

```bash
cd /tmp
md5sum -c zygsdk_2.0.0.7_aarch64.tar.gz.md5
```

`quick_deploy_scp.sh` 会在远程解压前自动校验。

### 6.1 安装前自动停进程

`install.sh install` 默认会先停止：

1. systemd 服务 `zygsdk`（若已注册且运行中）
2. 否则 `pkill` 匹配 `{安装目录}/bin/gsdk_basic_example` 的进程

跳过停进程：

```bash
ZYGSDK_SKIP_STOP=1 ./install.sh install /opt/zygsdk
```

### 6.2 tar 时间戳 “in the future”

**现象（远程解压时）：**

```text
tar: zygsdk_2.0.0.7_aarch64/xxx: time stamp 2026-xx-xx ... is ... s in the future
```

**原因：** WSL/开发机时间快于 NanoPC-T4。

**影响：** 一般不影响解压与安装。

**修复（设备上）：**

```bash
date
sudo timedatectl set-ntp true
```

---

### 6.3 sshpass 未安装

**现象（开发机部署时）：**

```text
提示: 已配置 DEVICE_PASSWORD 但未安装 sshpass，将尝试交互输入密码
```

**处理（WSL 开发机，可选）：**

```bash
sudo apt install sshpass
```

或配置 SSH 密钥免密，并将 `deploy/device.conf` 中 `DEVICE_PASSWORD` 留空。

---

## 七、修改配置文件

配置文件路径：`/opt/zygsdk/bin/config/config.json`

### 7.1 查看当前配置

```bash
cat /opt/zygsdk/bin/config/config.json
```

**重点字段：**

| 字段 | 说明 |
|------|------|
| `drone.ip` | 无人机 IP |
| `drone.port` | 通信端口（默认 9003） |
| `fpv` | FPV 视频流 |
| `gimbals` | 吊舱视频流 |
| `cloud_setting` | 云端 WebSocket |

---

### 7.2 编辑方式（设备无 nano 时）

**方式 A — vi/vim（设备上推荐）：**

```bash
vi /opt/zygsdk/bin/config/config.json
# i 编辑 → Esc → :wq 保存
```

**方式 B — sed 快速改 IP：**

```bash
sed -i 's/"ip": "192.168.1.40"/"ip": "实际无人机IP"/' /opt/zygsdk/bin/config/config.json
grep -A2 '"drone"' /opt/zygsdk/bin/config/config.json
```

**方式 C — 开发机改好再上传（推荐）：**

```bash
# WSL
nano /path/to/GSDK/config/config.json
scp /path/to/GSDK/config/config.json root@192.168.1.140:/opt/zygsdk/bin/config/config.json
```

**方式 D — 安装 nano：**

```bash
apt update && apt install -y nano
nano /opt/zygsdk/bin/config/config.json
```

---

## 八、启动与运行验证

### 8.1 前台启动

```bash
/opt/zygsdk/run.sh
```

> 必须使用 `run.sh`，它会设置 `LD_LIBRARY_PATH`；不要直接运行 `./gsdk_basic_example`。

**预期：** 终端输出 SDK 初始化日志，无立即崩溃或 `error while loading shared libraries`。

---

### 8.2 后台启动（可选）

```bash
cd /opt/zygsdk
nohup ./run.sh > /tmp/zygsdk.log 2>&1 &
tail -f /tmp/zygsdk.log
```

---

### 8.3 运行后检查

```bash
# 进程
ps aux | grep gsdk_basic_example

# 运行数据（程序启动后自动创建）
ls -la /opt/zygsdk/data/
ls -la /opt/zygsdk/data/logs/ 2>/dev/null

# 日志
tail -f /opt/zygsdk/data/logs/*.log
```

---

## 九、验证检查清单

安装完成后逐项勾选：

```text
[ ] ssh root@<设备IP> 可登录
[ ] cat /opt/zygsdk/VERSION → 版本正确（如 2.0.0.7）
[ ] file .../gsdk_basic_example → ARM aarch64
[ ] ldd ... | grep not found → 无输出
[ ] config.json 已按现场修改（drone.ip 等）
[ ] /opt/zygsdk/run.sh 可启动
[ ] /opt/zygsdk/data/logs/ 有日志（运行后）
[ ] （可选）/opt/zygsdk/install.sh status 正常
```

---

## 十、快速命令汇总

**设备端（复制粘贴）：**

```bash
# === 安装验证 ===
ls -la /opt/zygsdk/
cat /opt/zygsdk/VERSION
file /opt/zygsdk/bin/gsdk_basic_example
ldd /opt/zygsdk/bin/gsdk_basic_example | grep "not found"

# === 配置 ===
cat /opt/zygsdk/bin/config/config.json
vi /opt/zygsdk/bin/config/config.json

# === 启动 ===
/opt/zygsdk/run.sh

# === 运行后 ===
ps aux | grep gsdk_basic_example
tail -f /opt/zygsdk/data/logs/*.log
```

**开发机（WSL）重新部署：**

```bash
cd /path/to/GSDK
cp deploy/device.conf.example deploy/device.conf   # 首次
./scripts/quick_deploy_scp.sh
```

---

## 十一、故障速查

| 现象 | 处理 |
|------|------|
| `install.sh: No such file or directory` | 手动 scp 补装，或重新部署新版安装包 |
| `nano: command not found` | 用 `vi` 或开发机 scp 上传 config.json |
| `Exec format error` | 在 x86 PC 上误运行了 aarch64 程序，应在 NanoPC-T4 运行 |
| `error while loading shared libraries` | 使用 `/opt/zygsdk/run.sh` 启动 |
| tar 时间戳 future 警告 | 同步设备 NTP：`timedatectl set-ntp true` |
| 连接无人机失败 | 检查 `drone.ip`、网络 ping 通性 |

---

## 十二、实测记录摘要（2025-12-25）

| 步骤 | 命令 | 结果 |
|------|------|------|
| 部署 | WSL `./scripts/quick_deploy_scp.sh` | ✅ 132MB 包上传并安装到 `/opt/zygsdk` |
| 版本 | `cat /opt/zygsdk/VERSION` | ✅ `2.0.0.7` |
| 架构 | `file .../gsdk_basic_example` | ✅ ARM aarch64 |
| 依赖 | `ldd ... \| grep not found` | ✅ 无缺失 |
| install.sh | `install.sh status` | ⚠️ 旧包未复制，已文档化修复方案 |
| 配置编辑 | `nano config.json` | ⚠️ 设备无 nano，改用 vi/scp |

---

*文档版本：与 GSDK `VERSION` 2.0.0.7 同步；安装脚本修复后重新部署将自动包含 `/opt/zygsdk/install.sh`。*
