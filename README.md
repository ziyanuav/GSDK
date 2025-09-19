## GSDK 接口说明：FlightController / PayloadController / SensorDataManager

本文档基于 `gsdk-example/include/GsdkManager/sdkInterface.h` 中接口声明，说明三类核心控制器的职责、方法含义、参数与返回值约定、典型用法与注意事项。文中涉及的结构体与枚举（如 `Position3D`、`Attitude`、`BatteryInfo`、`SensorData`、`Mission`、`FlightState`、`FlightMode`、`GimbalMode`、`VideoProtocol` 等）均由 SDK 其他头文件定义。

### 通用约定

- **返回值**：`bool` 返回类型的方法，`true` 表示请求已被 SDK 正确受理并开始执行；`false` 表示同步校验失败（参数非法、状态不允许、连接中断等）。异步执行结果请结合回调或状态查询方法确认。
- **线程安全**：除特殊说明外，接口可从业务线程调用；建议对同一控制器实例按序发起命令，避免并发写入造成冲突。
- **时间单位**：未特别标注的时间单位为秒；频率单位为 Hz。
- **坐标/角度**：位置通常使用 WGS84（经纬高）或本地系，具体以结构体定义为准；角度单位为度（°），遵循右手系。
- **错误定位**：调用失败时可结合 `SDKManager::getLastError()` 或日志模块获取原因。

---

### FlightController（飞控/任务）

负责解锁、起降、导航与任务控制等飞行相关功能。

| 方法 | 说明 |
|---|---|
| `bool arm()` | 解锁电机。前置条件：已连接、GPS/姿态状态满足飞行要求、未在禁飞/地理围栏限制内。|
| `bool takeOff(float height)` | 起飞到指定相对高度 `height`。单位米，需已解锁。|
| `bool guideToPosition(double latitude, double longitude, float altitude)` | 导航至指定经纬度与高度。高度单位米，坐标系与 SDK 配置一致。|
| `bool returnToHome()` | 触发返航流程。返航点由飞控/SDK 维护。|
| `bool land()` | 触发降落流程。|
| `bool uploadMission(const Mission& mission, const std::function<void(int current, int total)>& userProgressCallback)` | 上传任务，并通过 `userProgressCallback` 推送分片进度。返回 `true` 仅代表开始上传。|
| `bool startMission()` | 启动已上传任务。|
| `bool pauseMission()` | 暂停任务。|
| `bool resumeMission()` | 恢复任务。|
| `bool stopMission()` | 终止任务。|
| `int getMissionProgress() const` | 获取当前任务进度（0~100，或实现定义）。|
| `Mission getCurrentMission() const` | 获取当前任务描述（快照）。|
| `FlightState getFlightState() const` | 获取飞行状态（位置、姿态、速度、模式等）。|
| `float getHealthScore() const` | 获取系统健康评分（0.0~1.0 或实现定义）。|
| `SensorData getTelemetryData() const` | 获取飞行遥测数据聚合。|
| `bool setVirtualStick(float throttle, float yaw, float pitch, float roll)` | 发送虚拟摇杆指令。需启用虚拟摇杆模式。单位与范围由固件定义，建议在固定频率循环发送。|
| `bool enableVirtualStickMode(bool enable)` | 开关虚拟摇杆模式。开启后，飞控由手动/任务转为接受摇杆控制。|
| `bool setGeofence(const std::vector<Position3D>& polygon)` | 设置多边形地理围栏。坐标系与 `Position3D` 定义一致。|
| `bool setCircularGeofence(const Position3D& center, float radius)` | 设置圆形地理围栏，半径单位米。|
| `bool removeGeofence()` | 移除地理围栏限制。|
| `bool setFlightMode(FlightMode mode)` | 设置飞行模式（例如稳姿、自主、任务等）。|
| `FlightMode getCurrentFlightMode() const` | 获取当前飞行模式。|

使用建议与注意事项：
- **任务上传**：较大任务建议分段上传，使用进度回调观测；上传完成后再 `startMission()`。
- **虚拟摇杆**：开启后请以稳定周期（典型 20~50 Hz）发送 `setVirtualStick`，避免看门狗超时导致失控保护触发。
- **地理围栏**：变更围栏在飞行中可能影响路径规划，务必确认新边界合理且不过于逼近当前机体位置。

示例：起飞并导航

```cpp
auto fc = core->getFlightController();
if (!fc->arm()) { /* 处理失败 */ }
if (!fc->takeOff(20.0f)) { /* 处理失败 */ }
// 导航至目标点
fc->guideToPosition(31.2304, 121.4737, 80.0f);
```

---

### PayloadController（载荷/云台/相机）

负责云台姿态/模式、变焦与拍照录像等媒体控制，以及自定义部件数据交互。

| 方法 | 说明 |
|---|---|
| `bool setGimbalAttitude(float pitch, float yaw)` | 设置云台俯仰与偏航角（度）。范围受硬件约束。|
| `bool setGimbalMode(GimbalMode mode)` | 设置云台模式（如跟随/锁定等）。|
| `bool calibrateGimbal()` | 启动云台校准流程。|
| `bool resetGimbal()` | 重置云台到默认姿态。|
| `bool opticalZoom(float length)` | 设置光学变焦焦距/倍数（实现定义）。|
| `bool opticalSpeedZoom(float speed)` | 以速度控制光学变焦，`speed` 正负表示放大/缩小。|
| `bool digitalZoom(float zoom)` | 设置数字变焦倍数。|
| `bool takePhoto()` | 触发拍照。|
| `bool startRecording()` | 开始录像。|
| `bool stopRecording()` | 停止录像。|
| `void getCustomWidgetData(std::function<void(const json&)> callback)` | 异步获取自定义部件数据，完成后通过回调返回 JSON。|
| `bool setCustomWidgetValue(uint8_t index, const std::string& widget_index, uint8_t type, uint8_t value)` | 设置自定义控件值；`type/value` 语义由部件定义。|
| `void subscribeGimbalAttitude(std::function<void(const json&)> callback)` | 订阅云台姿态变化（JSON 载荷，频率由实现定义）。|

注意事项：
- 录制/拍照状态切换需考虑存储介质与写入速度；在 `startRecording()` 成功后再进行连续控制。
- 光学与数字变焦能力依赖具体相机模组，超出范围会返回失败。

示例：调整云台并拍照

```cpp
auto pc = core->getPayloadController();
pc->setGimbalMode(GimbalMode::Follow);
pc->setGimbalAttitude(-45.0f, 30.0f);
pc->takePhoto();
```

---

### SensorDataManager（传感/遥测）

提供当前位置、姿态、电池与 GNSS 等传感数据访问与订阅能力。

| 方法 | 说明 |
|---|---|
| `SensorData getCurrentSensorData()` | 获取聚合传感数据快照。|
| `Position3D getCurrentPosition()` | 获取当前位置。|
| `Attitude getCurrentAttitude()` | 获取当前姿态。|
| `BatteryInfo getCurrentBatteryInfo()` | 获取电池信息（电量、电压、温度等）。|
| `float getCurrentAltitude()` | 获取当前海拔/相对高度（实现定义）。|
| `bool subscribeToSensorData(SensorDataCallback callback, int frequencyHz = 5)` | 订阅传感数据推送，频率上限由设备能力与链路带宽决定。|
| `bool unsubscribeFromSensorData()` | 取消订阅。|
| `bool setDataPushFrequency(int frequencyHz)` | 设置默认数据推送频率（对后续订阅生效或全局生效，视实现）。|
| `std::vector<float> getIMUData()` | 获取 IMU 原始/处理后数据（向量内容实现定义）。|
| `std::vector<float> getGPSData()` | 获取 GPS 原始/处理后数据（向量内容实现定义）。|
| `int getSatelliteCount()` | 当前可用卫星数量。|
| `bool isGPSFixed()` | 是否达到定位固定（如 3D Fix/RTK Fix）。|

使用建议：
- **订阅频率**：在边缘设备上 5~20 Hz 较为常见；如需更高频率，应评估 CPU/带宽占用并尽量只订阅必需字段。
- **数据一致性**：聚合快照非实时连续；对于控制回路，优先使用订阅流中的时间戳对齐数据。

示例：订阅遥测

```cpp
auto sm = core->getSensorDataManager();
sm->subscribeToSensorData(
  [](const SensorData& data){ /* 处理数据 */ },
  10 /* Hz */
);
```

---

### 常见流程范式

1) 起飞—执行任务—返航

```cpp
auto fc = core->getFlightController();
auto sm = core->getSensorDataManager();

if (fc->arm() && fc->takeOff(30.0f)) {
  fc->uploadMission(mission, [](int cur, int tot){});
  fc->startMission();
  // 订阅监控
  sm->subscribeToSensorData([](const SensorData& s){ /* 监控 */ }, 10);
}
```

2) FPV/云台联动拍摄

```cpp
auto pc = core->getPayloadController();
pc->setGimbalMode(GimbalMode::Lock);
pc->startRecording();
pc->opticalSpeedZoom(+0.5f);
```

---

### 错误处理与诊断建议

- **调用前置检查**：确认 `SDKManager::isConnected()` 为 `true`，并校验飞行/载荷状态是否允许操作。
- **失败重试**：短暂链路抖动或状态过渡可能导致失败，建议指数退避重试，并设置最大重试次数。
- **日志/诊断**：结合 `LogAndDiagnostics` 与系统日志导出定位问题；在关键调用前后打印上下文（模式、状态、位置等）。

---




