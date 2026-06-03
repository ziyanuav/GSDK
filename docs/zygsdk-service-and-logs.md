# 紫燕 GSDK — 服务管理与日志查看

本文档提供 NanoPC-T4 上 **zygsdk** 的 systemd 服务管理命令与日志查看方法。

> 前提：安装路径默认 `/opt/zygsdk`  
> 关联：[部署与升级指南](zygsdk-aarch64-deploy-upgrade.md) | [安装验证记录](zygsdk-aarch64-install-verify.md)

---

## 一、首次注册 systemd 与 logrotate（一次性）

安装 GSDK 后，在设备上执行：

```bash
cd /tmp/zygsdk_2.0.0.7_aarch64   # 或当前解压目录
chmod +x setup_service.sh
sudo ./setup_service.sh /opt/zygsdk
```

或从安装目录（若已复制脚本）：

```bash
sudo /opt/zygsdk/setup_service.sh /opt/zygsdk
```

**手动安装（等效）：**

```bash
sudo cp zygsdk.service /etc/systemd/system/zygsdk.service
sudo cp zygsdk.logrotate /etc/logrotate.d/zygsdk
sudo mkdir -p /opt/zygsdk/bin/logs /opt/zygsdk/data/logs
sudo systemctl daemon-reload
```

**logrotate 策略（`/etc/logrotate.d/zygsdk`）：**

| 项 | 值 |
|----|-----|
| 日志路径 | `/opt/zygsdk/bin/logs/*.log`、`/opt/zygsdk/data/logs/*.log` |
| 单文件上限 | **100MB**（`size 100M`） |
| 保留天数 | **14 天**（`rotate 14` + `daily`） |
| 压缩 | 启用（`compress` + `delaycompress`） |

---

## 二、服务管理（systemd）

服务名默认：**`zygsdk`**

### 2.1 常用命令

```bash
# 启动
sudo systemctl start zygsdk

# 停止
sudo systemctl stop zygsdk

# 重启
sudo systemctl restart zygsdk

# 查看状态
sudo systemctl status zygsdk

# 查看是否开机自启
sudo systemctl is-enabled zygsdk

# 开启开机自启
sudo systemctl enable zygsdk

# 关闭开机自启
sudo systemctl disable zygsdk
```

### 2.2 状态输出说明

`systemctl status zygsdk` 中关注：

| 字段 | 正常值 |
|------|--------|
| `Active` | `active (running)` |
| `Loaded` | `enabled`（若已开自启） |
| `Main PID` | 有进程号 |

### 2.3 未注册 systemd 时的手动启停

```bash
# 启动（前台）
/opt/zygsdk/run.sh

# 后台启动
cd /opt/zygsdk && nohup ./run.sh >> /opt/zygsdk/bin/logs/console.log 2>&1 &

# 停止
pkill -f '/opt/zygsdk/bin/gsdk_basic_example'

# 查看是否在运行
ps aux | grep gsdk_basic_example | grep -v grep
```

---

## 三、日志查看

### 3.1 日志路径说明

| 类型 | 路径 | 说明 |
|------|------|------|
| SDK 应用日志 | `/opt/zygsdk/bin/logs/` | 程序 `logPath=./logs`（工作目录为 bin） |
| 运行时数据日志 | `/opt/zygsdk/data/logs/` | SDK 运行后自动生成（若有） |
| systemd 日志 | `journalctl -u zygsdk` | 标准输出/错误（使用 systemd 时） |
| 控制台重定向 | `/opt/zygsdk/bin/logs/console.log` | 手动 nohup 时可选 |

**列出当前日志文件：**

```bash
ls -lh /opt/zygsdk/bin/logs/
ls -lh /opt/zygsdk/data/logs/ 2>/dev/null
```

---

### 3.2 systemd 日志（journalctl）

```bash
# 实时查看（Ctrl+C 退出）
sudo journalctl -u zygsdk -f

# 查看最近 50 行
sudo journalctl -u zygsdk -n 50 --no-pager

# 查看今天的日志
sudo journalctl -u zygsdk --since today

# 查看指定时间段
sudo journalctl -u zygsdk --since "2025-12-25 08:00:00" --until "2025-12-25 18:00:00"

# 查看本次启动以来的日志
sudo journalctl -u zygsdk -b
```

---

### 3.3 应用日志文件（/opt/zygsdk/bin/logs/）

将 `APP_LOG` 设为实际文件名（如 `zygsdk.log`），或直接用通配符：

```bash
# 实时查看（Ctrl+C 退出）
tail -f /opt/zygsdk/bin/logs/*.log

# 查看最近 50 行
tail -n 50 /opt/zygsdk/bin/logs/*.log

# 查看今天的日志（按文件名日期后缀，logrotate 后为 *.log-YYYYMMDD）
grep -h "" /opt/zygsdk/bin/logs/*.log-$(date +%Y%m%d) 2>/dev/null \
  || grep -h "$(date +%Y-%m-%d)" /opt/zygsdk/bin/logs/*.log 2>/dev/null \
  || tail -n 200 /opt/zygsdk/bin/logs/*.log

# 查看指定时间段（示例：12月25日 08:00–18:00）
awk '$0 >= "2025-12-25 08:00:00" && $0 <= "2025-12-25 18:00:00"' \
  /opt/zygsdk/bin/logs/*.log 2>/dev/null

# 或按 journal 时间过滤 + 文件（若日志含 ISO 时间戳）
sed -n '/2025-12-25 08:/,/2025-12-25 18:/p' /opt/zygsdk/bin/logs/*.log
```

**data 目录日志（若有）：**

```bash
tail -f /opt/zygsdk/data/logs/*.log
tail -n 50 /opt/zygsdk/data/logs/*.log
```

---

### 3.4 日志轮转验证

```bash
# 查看 logrotate 配置
cat /etc/logrotate.d/zygsdk

# 手动触发一次轮转（测试用）
sudo logrotate -d /etc/logrotate.d/zygsdk    # 调试，不实际轮转
sudo logrotate -f /etc/logrotate.d/zygsdk    # 强制轮转

# 查看轮转后的备份文件
ls -lh /opt/zygsdk/bin/logs/
# 可能出现：app.log  app.log-20251225.gz  ...
```

---

## 四、推荐生产部署流程

```bash
# 1. 安装 GSDK
cd /tmp/zygsdk_2.0.0.7_aarch64
sudo ./install.sh install /opt/zygsdk

# 2. 修改配置
vi /opt/zygsdk/bin/config/config.json

# 3. 注册服务与 logrotate
sudo ./setup_service.sh /opt/zygsdk

# 4. 开机自启并启动
sudo systemctl enable zygsdk
sudo systemctl start zygsdk

# 5. 验证
sudo systemctl status zygsdk
sudo journalctl -u zygsdk -n 20 --no-pager
ls -lh /opt/zygsdk/bin/logs/
```

---

## 五、与 install.sh 的联动

`install.sh` 升级/回滚时会：

1. 优先 `systemctl stop zygsdk`（若服务已注册）
2. 否则 `pkill` 停止 `gsdk_basic_example`

升级后需手动或脚本重启服务：

```bash
sudo systemctl restart zygsdk
```

---

## 六、故障速查

| 现象 | 处理 |
|------|------|
| `Unit zygsdk.service not found` | 执行 `setup_service.sh` 注册服务 |
| `status` 显示 failed | `journalctl -u zygsdk -n 100 --no-pager` 查原因 |
| 无 `/opt/zygsdk/bin/logs/` | 先启动一次服务；`mkdir -p /opt/zygsdk/bin/logs` |
| 日志文件过大 | 确认 `/etc/logrotate.d/zygsdk` 已安装 |
| 停服务后进程仍在 | `sudo systemctl stop zygsdk` 或 `pkill -f /opt/zygsdk/bin/gsdk_basic_example` |

---

## 七、命令速查表

```bash
# --- 服务 ---
sudo systemctl start|stop|restart|status zygsdk
sudo systemctl is-enabled zygsdk
sudo systemctl enable|disable zygsdk

# --- journal 日志 ---
sudo journalctl -u zygsdk -f
sudo journalctl -u zygsdk -n 50 --no-pager
sudo journalctl -u zygsdk --since today
sudo journalctl -u zygsdk --since "YYYY-MM-DD HH:MM:SS" --until "YYYY-MM-DD HH:MM:SS"

# --- 文件日志 ---
tail -f /opt/zygsdk/bin/logs/*.log
tail -n 50 /opt/zygsdk/bin/logs/*.log
ls -lh /opt/zygsdk/bin/logs/
```
