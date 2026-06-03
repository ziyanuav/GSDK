# 紫燕 GSDK (Ziyan GSDK) — NanoPC-T4 (aarch64) 部署与升级指南

本文档面向 **NanoPC-T4 + Ubuntu 20.04/22.04**，说明紫燕 GSDK 的首次安装、升级、回滚、日常运维及快速开发部署流程。

> **安装后验证：** 见 [安装验证与操作记录](zygsdk-aarch64-install-verify.md)

---

## 一、环境与前提

| 项目 | 说明 |
|------|------|
| 目标设备 | NanoPC-T4（RK3399，aarch64） |
| 操作系统 | Ubuntu 20.04 / 22.04（实测 20.04.6 可用） |
| 默认安装路径 | `/opt/zygsdk`（可在配置文件中修改） |
| 开发机 | x86_64 Linux / WSL，可交叉编译 |
| 设备连接 | 通过 `deploy/device.conf` 配置 IP / 账号 / 密码 |
| 发布包命名 | `zygsdk_<版本>_aarch64.tar.gz`，例如 `zygsdk_2.0.0.7_aarch64.tar.gz` |

---

## 二、设备连接配置（IP / 账号 / 密码）

不同现场的 NanoPC-T4 **IP、账号、密码可能不同**，请使用本地配置文件，**不要写死在脚本里**。

### 2.1 创建本地配置(可先查看是否存在device.conf，没有就创建)

```bash
cd /path/to/GSDK
cp deploy/device.conf.example deploy/device.conf
nano deploy/device.conf
```

`deploy/device.conf` 已加入 `.gitignore`，不会被提交到 Git。

**示例 `deploy/device.conf`：**

```bash
DEVICE_HOST=192.168.1.140      # 设备 IP 或主机名
DEVICE_USER=root               # SSH 用户
DEVICE_PORT=22                 # SSH 端口
DEVICE_PASSWORD=               # 可选；留空则用密钥或交互输入
# SSH_KEY=~/.ssh/id_rsa        # 可选，指定私钥
REMOTE_TMP=/tmp                # 上传 tar.gz 的远程目录
INSTALL_DIR=/opt/zygsdk        # 设备安装路径
```

### 2.2 配置优先级

| 优先级 | 方式 | 示例 |
|--------|------|------|
| 1（最高） | 命令行参数 | `./scripts/quick_deploy_scp.sh 192.168.1.140` |
| 2 | 环境变量 | `export ZYGSDK_DEVICE_HOST=192.168.1.140` |
| 3 | 指定配置文件 | `export ZYGSDK_DEPLOY_CONF=~/my_t4.conf` |
| 4 | 默认本地文件 | `deploy/device.conf` |
| 5 | 用户级配置 | `~/.config/zygsdk/device.conf` |
| 6 | 示例模板 | `deploy/device.conf.example` |

**常用环境变量：**

```bash
export ZYGSDK_DEVICE_HOST=192.168.1.140
export ZYGSDK_DEVICE_USER=root
export ZYGSDK_DEVICE_PASSWORD=your_password   # 需安装 sshpass 才能非交互
export ZYGSDK_INSTALL_DIR=/opt/zygsdk
```

### 2.3 密码与免密登录

- **推荐**：配置 SSH 密钥免密，`DEVICE_PASSWORD` 留空
- **可选**：填写 `DEVICE_PASSWORD` 并安装 `sshpass`（`sudo apt install sshpass`），脚本可非交互部署
- **未配置密码且无密钥**：执行 `scp`/`ssh` 时会提示交互输入密码

---

## 三、方案说明：为何使用 tar.gz

**结论：合理，推荐使用 tar.gz 作为标准安装包格式。**

| 对比项 | tar.gz | rar / zip |
|--------|--------|-----------|
| Linux 原生支持 | `tar` 系统自带 | 需额外安装 unrar/unzip |
| 可执行权限 | 保留（配合 `chmod`） | rar/zip 解压常丢失 |
| 符号链接 | 打包前用 `cp -RL` 解引用，稳定 | NTFS/U 盘易出现 0 字节 .so |
| 升级/回滚 | 配合 `install.sh` 自动备份 | 需手工操作 |
| 离线 U 盘 | 支持 | 支持 |

**标准发布流程（4 步）：**

```text
(1) cd /path/to/GSDK
(2) ./scripts/build_gsdk_example.sh aarch64 clean   # 或由发布脚本自动执行
(3) ./scripts/build_zygsdk_release.sh [版本号]        # 生成 dist/zygsdk_x.x.x.x_aarch64.tar.gz
(4) 传输到设备 → 解压 → sudo ./install.sh install
```

---

## 四、开发机构建发布包

### 4.1 首次准备（一次性）

```bash
cd /path/to/GSDK

# 若 lib/aarch64/ffmpeg 为 Android 版，需先编译 Linux 版（仅需一次）
chmod +x scripts/build_ffmpeg_aarch64_linux.sh
./scripts/build_ffmpeg_aarch64_linux.sh
```

### 4.2 修改版本号

编辑项目根目录 `VERSION` 文件，例如：

```text
2.0.0.7
```

### 4.3 一键构建 tar.gz

```bash
chmod +x scripts/build_zygsdk_release.sh
./scripts/build_zygsdk_release.sh
# 或指定版本与输出目录
./scripts/build_zygsdk_release.sh 2.0.0.7 ./dist
```

**产物：**

```text
dist/zygsdk_2.0.0.7_aarch64.tar.gz
dist/zygsdk_2.0.0.7_aarch64.tar.gz.md5   # MD5 校验文件（自动生成）
```

**设备解压前校验（U 盘 / 手动 SCP 场景）：**

```bash
cd /tmp
md5sum -c zygsdk_2.0.0.7_aarch64.tar.gz.md5
tar xzf zygsdk_2.0.0.7_aarch64.tar.gz
```

**包内结构：**

```text
zygsdk_2.0.0.7_aarch64/
├── VERSION              # 版本号
├── manifest.txt         # 构建信息
├── install.sh           # 安装/升级/回滚
├── run.sh               # 启动脚本
├── setup.sh             # 修复权限
├── bin/
│   ├── gsdk_basic_example
│   └── config/config.json
└── lib/aarch64/         # 全部运行时库
```

### 4.4 局域网快速部署（开发调试）

```bash
# 先配置 deploy/device.conf，然后一条命令完成 构建 + 上传 + 安装
chmod +x scripts/quick_deploy_scp.sh
./scripts/quick_deploy_scp.sh

# 临时指定 IP（覆盖配置文件）
./scripts/quick_deploy_scp.sh 192.168.1.140 2.0.0.7
```

该脚本会自动：构建 → SCP 上传 → SSH 远程安装到 `INSTALL_DIR`（默认 `/opt/zygsdk`）。

仅上传已有安装包（不重新编译）：

```bash
./scripts/deploy_upload.sh
# 或指定路径
./scripts/deploy_upload.sh dist/zygsdk_2.0.0.7_aarch64.tar.gz
```

---

## 五、首次安装

### 方式 A：SCP 直传（有局域网）

**在开发机（WSL / Linux）：**

```bash
# 读取 deploy/device.conf 中的 DEVICE_HOST / DEVICE_USER
source scripts/load_deploy_config.sh && load_deploy_config
scp dist/zygsdk_2.0.0.7_aarch64.tar.gz ${DEVICE_USER}@${DEVICE_HOST}:${REMOTE_TMP}/

# 或直接使用上传脚本
./scripts/deploy_upload.sh
```

**在 NanoPC-T4：**

```bash
ssh ${DEVICE_USER}@${DEVICE_HOST}   # 账号/IP 以 device.conf 为准

cd /tmp
tar xzf zygsdk_2.0.0.7_aarch64.tar.gz
cd zygsdk_2.0.0.7_aarch64
chmod +x install.sh run.sh setup.sh bin/gsdk_basic_example
sudo ./install.sh install /opt/zygsdk
```

**在 Windows PowerShell（仅传文件）：**

```powershell
# 将 <USER>、<HOST> 替换为 device.conf 中的值
scp D:\path\to\zygsdk_2.0.0.7_aarch64.tar.gz <USER>@<HOST>:/tmp/
ssh <USER>@<HOST>
```

### 方式 B：U 盘拷贝（完全离线）

1. 将 `zygsdk_2.0.0.7_aarch64.tar.gz` 复制到 U 盘
2. U 盘插入 NanoPC-T4，挂载后复制到 `/tmp/`
3. 执行与方式 A 相同的解压、安装命令

> **注意**：不要使用 rar；若从 Windows 拷贝目录而非 tar.gz，`.so` 符号链接可能变成 0 字节文件。

### 5.1 修改配置并启动

```bash
# 按现场环境修改无人机 IP、视频流等
sudo nano /opt/zygsdk/bin/config/config.json

# 启动
/opt/zygsdk/run.sh
```

程序运行后会在 `/opt/zygsdk/data/` 生成日志与数据库。

---

## 六、升级操作

```bash
# 1. 开发机生成新版本包
./scripts/build_zygsdk_release.sh 2.0.0.8

# 2. 一键升级（推荐，使用 device.conf）
./scripts/quick_deploy_scp.sh

# 或分步：仅上传 + 设备上手动安装
./scripts/deploy_upload.sh
ssh ${DEVICE_USER}@${DEVICE_HOST}
cd /tmp
tar xzf zygsdk_2.0.0.8_aarch64.tar.gz
cd zygsdk_2.0.0.8_aarch64
sudo ./install.sh install /opt/zygsdk
```

**升级策略（默认）：**

- 安装前**自动停止**运行中的 `gsdk_basic_example`（或 systemd 服务 `zygsdk`）
- 自动备份旧版本到 `/opt/zygsdk_backup/zygsdk_<旧版本>_<时间戳>/`
- **保留** 原 `config.json` 与 `data/` 目录
- 可用环境变量控制：`ZYGSDK_KEEP_CONFIG=0` 强制覆盖配置；`ZYGSDK_SKIP_STOP=1` 升级时不停止旧进程

---

## 七、版本回滚

```bash
# 查看状态
sudo /opt/zygsdk/install.sh status /opt/zygsdk
# 或进入最近一次解压目录
sudo ./install.sh status /opt/zygsdk

# 回滚到上一版本
sudo ./install.sh rollback /opt/zygsdk

# 启动验证
/opt/zygsdk/run.sh
```

手动回滚（若自动回滚失败）：

```bash
ls /opt/zygsdk_backup/
sudo rm -rf /opt/zygsdk
sudo cp -a /opt/zygsdk_backup/zygsdk_2.0.0.7_YYYYMMDD_HHMMSS /opt/zygsdk
/opt/zygsdk/run.sh
```

---

## 八、日常运维

| 操作 | 命令 |
|------|------|
| 启动 | `sudo systemctl start zygsdk` 或 `/opt/zygsdk/run.sh` |
| 停止 | `sudo systemctl stop zygsdk` |
| 服务状态 | `sudo systemctl status zygsdk` |
| 查看版本 | `cat /opt/zygsdk/VERSION` |
| 查看日志 | 见 [服务管理与日志](zygsdk-service-and-logs.md) |
| 查看数据库 | `ls /opt/zygsdk/data/zygsdk.db` |
| 修改配置 | `nano /opt/zygsdk/bin/config/config.json` 后重启 |
| 检查依赖 | `ldd /opt/zygsdk/bin/gsdk_basic_example` |
| 卸载 | `sudo ./install.sh uninstall /opt/zygsdk` |

**建议：**

- 升级前备份 `config.json` 与 `data/`
- 生产环境固定安装路径 `/opt/zygsdk`
- 不要在 x86 PC 上直接运行 aarch64 二进制（会报 `Exec format error`）

---

## 九、快速开发部署流程（同网段）

```mermaid
flowchart LR
  A[开发机 GSDK] --> B[build_zygsdk_release.sh]
  B --> C[zygsdk_x.x.x.x_aarch64.tar.gz]
  C --> D{传输方式}
  D -->|SCP| E[NanoPC-T4 /tmp]
  D -->|U盘| E
  E --> F[install.sh install]
  F --> G[/opt/zygsdk/run.sh]
```

**一条命令（需先配置 `deploy/device.conf`）：**

```bash
./scripts/quick_deploy_scp.sh
```

---

## 十、常见问题

### 1. `Exec format error`

在 x86 开发机运行了 aarch64 程序。请在 NanoPC-T4 上运行。

### 2. `Permission denied`（run.sh）

```bash
chmod +x /opt/zygsdk/run.sh /opt/zygsdk/bin/gsdk_basic_example
# 或
/opt/zygsdk/setup.sh
```

### 3. `libc.so: invalid ELF header`

ffmpeg 仍为 Android 版。开发机执行 `./scripts/build_ffmpeg_aarch64_linux.sh` 后重新打包。

### 4. `ld.so: dl-version.c: Assertion failed`

曾对 Android ffmpeg 使用 patchelf 修补。应换 Linux glibc 版 ffmpeg，不要用 patchelf 改 Android 库。

### 5. `undefined symbol: CrashHandler::UnwindCallback`

`libZiyan.so` 过旧。请使用新版 SDK 库并重新打包；或确认库内已包含 CrashHandler。

### 6. 库文件 0 字节

从 Windows 直接复制目录导致符号链接损坏。必须使用 `tar.gz` 安装包，或开发机 `cp -RL` 后打包。

### 7. 验证 ffmpeg 是否为 Linux 版

```bash
readelf -d /opt/zygsdk/lib/aarch64/ffmpeg/libavcodec.so | grep NEEDED
# 应看到 libc.so.6，不是 libc.so
```

---

## 十一、脚本索引

| 脚本 | 用途 |
|------|------|
| `deploy/device.conf.example` | 设备连接配置模板（复制为 `device.conf`） |
| `scripts/load_deploy_config.sh` | 加载设备配置（被其他脚本引用） |
| `scripts/build_gsdk_example.sh aarch64 clean` | 交叉编译 |
| `scripts/build_ffmpeg_aarch64_linux.sh` | 编译 Linux ffmpeg（首次） |
| `scripts/build_zygsdk_release.sh [版本]` | **构建 tar.gz 安装包** |
| `scripts/quick_deploy_scp.sh [IP] [版本]` | 构建 + SCP + 远程安装 |
| `scripts/deploy_upload.sh [tar.gz]` | 仅上传安装包（不编译） |
| `scripts/device/install_zygsdk.sh` | 设备端安装/升级/回滚（打入安装包，安装至 `/opt/zygsdk/install.sh`） |
| `docs/zygsdk-aarch64-install-verify.md` | **安装后验证与操作记录** |

---

## 十二、检查清单

**发布前（开发机）：**

- [ ] `deploy/device.conf` 已按现场配置 IP/账号
- [ ] `VERSION` 已更新
- [ ] `./scripts/build_zygsdk_release.sh` 成功
- [ ] `dist/zygsdk_*_aarch64.tar.gz` 及 `.md5` 已生成
- [ ] ffmpeg 依赖 `libc.so.6`

**安装后（NanoPC-T4）：**

- [ ] 按 [安装验证与操作记录](zygsdk-aarch64-install-verify.md) 完成验证
- [ ] `cat /opt/zygsdk/VERSION` 版本正确
- [ ] `config.json` 已按现场修改
- [ ] `/opt/zygsdk/run.sh` 正常启动
- [ ] `data/logs/` 有日志输出
