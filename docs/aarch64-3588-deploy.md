# GSDK aarch64 部署指南（RK3588 等 ARM64 Linux）

本文档说明如何将 `gsdk_basic_example` 交叉编译并部署到 ARM64 设备（如 RK3588）上运行。

---

## 一、环境说明

| 项目 | 说明 |
|------|------|
| 开发机 | x86_64 Linux / WSL（用于交叉编译） |
| 目标设备 | ARM64（aarch64）Linux，如 RK3588 |
| 工具链 | 工程内 `Tool/bin/aarch64-buildroot-linux-gnu-gcc` |
| 运行时库 | 工程内 `lib/aarch64/` |

> **注意**：在 x86 开发机上无法直接运行 aarch64 可执行文件；若出现 `Exec format error`，说明当前产物是 ARM 版本，需在目标设备上运行，或重新编译 x86 版本用于本机调试。

---

## 二、交叉编译

在工程根目录执行：

```bash
./scripts/build_gsdk_example.sh aarch64 clean
```

成功后可执行文件位于：

```
build/bin/gsdk_basic_example
```

验证架构：

```bash
file build/bin/gsdk_basic_example
# 应显示: ARM aarch64
```

---

## 三、需要部署到目标设备的内容

### 必须拷贝

| 路径 | 说明 |
|------|------|
| `build/bin/gsdk_basic_example` | ARM64 主程序 |
| `config/`（或 `build/bin/config/`） | 配置文件，含 `config.json` |
| `lib/aarch64/` | 全部运行时依赖库（约 270MB） |

### 不需要拷贝

| 路径 | 原因 |
|------|------|
| `include/` | 头文件，运行不需要 |
| `Tool/` | 交叉编译工具链，仅开发机使用 |
| `build/` 中其他文件 | 构建中间产物 |
| `data/` | 程序首次运行后自动创建（日志、数据库） |

### 运行时依赖概览

程序通过 RPATH / `LD_LIBRARY_PATH` 加载 `lib/aarch64/` 下各子目录中的库，主要包括：

- 主库：`libZiyan.so`
- 网络 / 加密：`libssl.so.3`、`libcrypto.so.3`、`libcurl.so.4`、`libevent`（`event/lib/`）
- 媒体：`ffmpeg/`（`libavcodec`、`libavformat` 等）
- 其他：`boost/`、`datachannel/lib/`、`3rd/`、`pcre2/` 等

---

## 四、推荐部署目录结构

将以下内容保持如下相对路径，整体拷贝到目标设备（例如 `/opt/gsdk`）：

```
gsdk/
├── run.sh                  # 启动脚本（推荐）
├── README.txt              # 简要说明（打包时自动生成）
├── bin/
│   ├── gsdk_basic_example  # 主程序
│   └── config/
│       └── config.json     # 运行配置
└── lib/
    └── aarch64/            # 全部运行时 .so
        ├── libZiyan.so
        ├── ffmpeg/
        ├── boost/
        ├── event/
        │   └── lib/        # 注意：aarch64 的 libevent 在此目录
        ├── datachannel/
        │   └── lib/
        └── ...
```

---

## 五、一键打包（开发机）

工程提供打包脚本，自动完成复制、RPATH 修正，并生成 `run.sh`：

```bash
# 1. 先交叉编译
./scripts/build_gsdk_example.sh aarch64 clean

# 2. 打包到默认目录（Windows D 盘示例路径）
./scripts/package_aarch64_deploy.sh
```

默认输出目录：

```
/mnt/d/上云资料/GSDK资料/3588部署/gsdk
```

也可指定自定义输出路径：

```bash
./scripts/package_aarch64_deploy.sh /path/to/output/gsdk
```

打包脚本会：

1. 复制 `bin/gsdk_basic_example`、`config/`、`lib/aarch64/`
2. 用 `patchelf` 将 RPATH 改为 `$ORIGIN/../lib/aarch64/...` 相对路径
3. 生成 `run.sh` 和 `README.txt`

---

## 六、在 RK3588 上部署与运行

### 1. 拷贝文件

将整个 `gsdk` 文件夹通过 U 盘、scp、adb 等方式复制到设备，例如：

```bash
scp -r gsdk/ user@192.168.x.x:/opt/gsdk
```

### 2. 赋予执行权限

```bash
chmod +x /opt/gsdk/run.sh
chmod +x /opt/gsdk/bin/gsdk_basic_example
```

### 3. 修改配置

编辑 `bin/config/config.json`，按现场环境修改：

- `drone.ip` / `drone.port`：无人机地址
- `fpv`、`gimbals`：视频流 URL
- `cloud_setting.url`：云端 WebSocket 地址
- `simulation`：`true` 仿真 / `false` 真机

### 4. 启动程序

**推荐方式**（使用启动脚本）：

```bash
/opt/gsdk/run.sh
```

**直接启动**：

```bash
cd /opt/gsdk/bin
./gsdk_basic_example
```

程序启动后会在运行目录附近自动创建 `data/` 目录，用于存放日志和数据库（如 `data/logs/`、`data/zygsdk.db`）。

---

## 七、RPATH 与库路径说明

### 为何需要处理 RPATH

交叉编译产物默认嵌入的是开发机绝对路径（如 `/home/liu/gsdk/GSDK/lib/aarch64/...`），拷贝到其他设备后无法找到库。

打包脚本会将 RPATH 修正为相对路径：

```
$ORIGIN/../lib/aarch64
$ORIGIN/../lib/aarch64/ffmpeg
$ORIGIN/../lib/aarch64/event/lib
...
```

其中 `$ORIGIN` 为可执行文件所在目录（`bin/`），因此只要保持 `bin/` 与 `lib/aarch64/` 的相对位置不变即可。

### 备用：`run.sh` 设置 `LD_LIBRARY_PATH`

若目标设备上 RPATH 仍有问题，可使用打包生成的 `run.sh`，其中已配置完整的 `LD_LIBRARY_PATH`。

---

## 八、常见问题

### 1. `cannot execute binary file: Exec format error`

在 x86 开发机上运行了 aarch64 程序。请在 ARM 设备上运行，或编译 x86 版本：

```bash
./scripts/build_gsdk_example.sh x86 clean
```

### 2. `error while loading shared libraries: libxxx.so`

- 确认 `lib/aarch64/` 已完整拷贝
- 使用 `run.sh` 启动
- 在设备上检查：`ldd /opt/gsdk/bin/gsdk_basic_example`

### 3. aarch64 的 libevent 路径

`lib/aarch64/event/` 根目录下可能混有 x86 库；**aarch64 版本在 `event/lib/`**。打包脚本与 RPATH 已指向正确路径。

### 4. 重新部署

修改代码或配置后，在开发机重新执行：

```bash
./scripts/build_gsdk_example.sh aarch64 clean
./scripts/package_aarch64_deploy.sh
```

再将新的 `gsdk` 目录覆盖到设备即可。

---

## 九、相关脚本

| 脚本 | 用途 |
|------|------|
| `scripts/build_gsdk_example.sh x86` | 本机 x86 编译与调试 |
| `scripts/build_gsdk_example.sh aarch64` | ARM64 交叉编译 |
| `scripts/package_aarch64_deploy.sh` | 生成可部署的 gsdk 目录 |

---

## 十、快速检查清单

- [ ] 开发机已执行 `aarch64 clean` 编译成功
- [ ] `file build/bin/gsdk_basic_example` 显示 `ARM aarch64`
- [ ] 已打包或手动拷贝 `bin/`、`config/`、`lib/aarch64/`
- [ ] 目标设备目录结构保持 `bin` 与 `lib/aarch64` 相对路径
- [ ] 已修改 `config.json` 中的 IP 与视频流地址
- [ ] 已 `chmod +x` 可执行文件
- [ ] 在 3588 上通过 `run.sh` 或直接进入 `bin/` 启动
