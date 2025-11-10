#ifndef __ZIYAN_API__
#define __ZIYAN_API__

#include <iostream>
#include <thread>
#include <mutex>
#include <nlohmann/json.hpp>
#include <csignal>
#include <string>
#include <vector>
#include <map>
#include <functional>
#include <memory>
#include <chrono>
#include <cstring>

using json = nlohmann::json;

// ================================
// 前向声明和类型别名
// ================================

// 基础数据结构（避免重复定义，使用前向声明）
struct Position3D;
struct Velocity3D;
struct Attitude;
struct BatteryInfo;
struct SensorData;
struct VideoStreamInfo;
struct SDKConfig;
struct Mission;
struct FlightState;

// 枚举类型（避免重复定义，使用前向声明）
// enum SensorType;  // 注释掉，因为ziyanSdk.h中已有定义
enum class SdkFlightMode;
enum class VideoProtocol;

// ================================
// 回调函数类型定义
// ================================

using StatusCallback = std::function<void(int status, const std::string &message)>;
using SensorDataCallback = std::function<void(const SensorData &data)>; // 重命名避免冲突
using ErrorCallback = std::function<void(int errorCode, const std::string &errorMsg)>;

// ================================
// 前向声明
// ================================

namespace DroneSDK
{
	class SDKManagerImpl;
	class FlightControllerImpl;
	class PayloadControllerImpl;
	class SensorDataManagerImpl;
	class CloudInterfaceImpl;
	class BatteryAndLinkManagerImpl;
	class VideoTransmissionImpl;
	class FlightParameterManager;
	class LogAndDiagnosticsImpl;
	class SDKCore;
	class EventListener;
	class SDKManager
	{
	public:
		virtual ~SDKManager() = default;
		virtual std::string getSDKVersion() const = 0;
		virtual std::string getCompatibilityInfo() const = 0;
		virtual std::string getUpdateLog() const = 0;
		virtual bool initialize(const std::string &config) = 0;
		virtual bool connect() = 0;
		virtual bool disconnect() = 0;
		virtual bool isConnected() const = 0;
		virtual int getNetworkStatus() const = 0;
		virtual int getInitializationProgress() const = 0;
		virtual std::string getLastError() const = 0;
		virtual void setStatusCallback(StatusCallback callback) = 0;
		virtual void setErrorCallback(ErrorCallback callback) = 0;
		virtual std::string getDeviceSerialNumber() const = 0;
	};
	// 飞行参数管理器接口 - 简化版本，只支持枚举类型
	class FlightParameterManager
	{
	public:
		virtual ~FlightParameterManager() = default;

		// 初始化
		virtual bool initialize() = 0;
		virtual void shutdown() = 0;

		// 读取参数 - 只支持枚举类型
		virtual void readParameter(FlightParameterType paramType, ParameterReadCallback callback) = 0;

		// 写入参数 - 只支持枚举类型
		virtual void writeParameter(FlightParameterType paramType, float value, ParameterWriteCallback callback) = 0;

		// 获取参数信息 - 只支持枚举类型
		virtual FlightParameterInfo getParameterInfo(FlightParameterType paramType) const = 0;

		// 参数验证 - 只支持枚举类型
		virtual bool validateParameterValue(FlightParameterType paramType, float value) const = 0;

		// 缓存管理 - 已移除，直接调用底层

		// 参数可用性检查 - 只支持枚举类型
		virtual bool isParameterAvailable(FlightParameterType paramType) const = 0;

		// 回调管理
		virtual void registerParameterChangeCallback(ParameterChangeCallback callback) = 0;
	};
	class FlightController
	{
	public:
		virtual ~FlightController() = default;
		virtual bool arm() = 0;
		virtual bool takeOff(float height) = 0;
		virtual bool guideToPosition(double latitude, double longitude, float altitude) = 0;
		virtual bool returnToHome() = 0;
		virtual bool land() = 0;
		virtual bool uploadMission(const Mission &mission, const std::function<void(int current, int total)> &userProgressCallback) = 0;
		virtual bool startMission() = 0;
		virtual bool pauseMission() = 0;
		virtual bool resumeMission() = 0;
		virtual bool stopMission() = 0;
		virtual int getMissionProgress() const = 0;
		virtual Mission getCurrentMission() const = 0;
		virtual FlightState getFlightState() const = 0;
		virtual float getHealthScore() const = 0;
		virtual SensorData getTelemetryData() const = 0;
		virtual bool setVirtualStick(float throttle, float yaw, float pitch, float roll) = 0;
		virtual bool enableVirtualStickMode(bool enable) = 0;
		virtual bool setGeofence(const std::vector<Position3D> &polygon) = 0;
		virtual bool setCircularGeofence(const Position3D &center, float radius) = 0;
		virtual bool removeGeofence() = 0;
		virtual bool setFlightMode(FlightModeControl mode) = 0;
		virtual FlightModeControl getCurrentFlightMode() const = 0;
	};

	class PayloadController
	{
	public:
		virtual ~PayloadController() = default;
		virtual bool setGimbalAttitude(float pitch, float yaw) = 0;
		virtual bool calibrateGimbal() = 0;
		virtual bool resetGimbal() = 0;
		virtual bool opticalZoom(float length) = 0;
		virtual bool opticalSpeedZoom(float speed) = 0;
		virtual bool digitalZoom(float zoom) = 0;
		virtual bool takePhoto() = 0;
		virtual bool startRecording() = 0;
		virtual bool stopRecording() = 0;
		virtual void getCustomWidgetData(std::function<void(const json &)> callback) = 0;
		virtual bool setCustomWidgetValue(const uint8_t index, const std::string &widget_index, const uint8_t type,
										  const uint8_t value) = 0;
		virtual void subscribeGimbalAttitude(std::function<void(const json &)> callback) = 0;
	};

	class SensorDataManager
	{
	public:
		virtual ~SensorDataManager() = default;
		virtual SensorData getCurrentSensorData() = 0;
		virtual Position3D getCurrentPosition() = 0;
		virtual Attitude getCurrentAttitude() = 0;
		virtual BatteryInfo getCurrentBatteryInfo() = 0;
		virtual float getCurrentAltitude() = 0;
		virtual bool subscribeToSensorData(SensorDataCallback callback, int frequencyHz = 5) = 0;
		virtual bool unsubscribeFromSensorData() = 0;
		virtual bool setDataPushFrequency(int frequencyHz) = 0;
		virtual std::vector<float> getIMUData() = 0;
		virtual std::vector<float> getGPSData() = 0;
		virtual int getSatelliteCount() = 0;
		virtual bool isGPSFixed() = 0;
	};

	class CloudInterface
	{
	public:
		virtual ~CloudInterface() = default;
		virtual bool uploadTelemetryData(const SensorData &data) = 0;
		virtual bool enableRealTimeUpload(bool enable) = 0;
		virtual bool sendCommandFromCloud(const std::string &command, const std::vector<uint8_t> &params) = 0;
		virtual bool syncDeviceStatus() = 0;
		virtual bool startVideoStream(const std::string &streamKey, VideoProtocol protocol) = 0;
		virtual bool stopVideoStream() = 0;
		virtual VideoStreamInfo getVideoStreamInfo() const = 0;
		virtual bool setStreamQuality(int bitrate, int framerate) = 0;
		virtual bool connectToCloud(const std::string &endpoint, const std::string &token) = 0;
		virtual bool disconnectFromCloud() = 0;
		virtual bool isCloudConnectionStatus() const = 0;
		virtual std::string getCloudStatus() const = 0;
		virtual bool setupWebSocketConnection(const std::string &url) = 0;
		virtual bool sendWebSocketMessage(const std::string &message) = 0;
		virtual void setWebSocketCallback(std::function<void(const std::string &)> callback) = 0;
	};

	class BatteryAndLinkManager
	{
	public:
		virtual ~BatteryAndLinkManager() = default;
		virtual BatteryInfo getBatteryInfo() = 0;
		virtual float getBatterySOH() = 0; // State of Health
		virtual float getBatterySOC() = 0; // State of Charge
		virtual bool isBatteryOverheated() = 0;
		virtual bool isLowBattery() = 0;
		virtual std::vector<float> getCellVoltages() = 0;
		virtual void setBatteryWarningCallback(std::function<void(const std::string &)> callback) = 0;
		virtual void setLowBatteryThreshold(float percentage) = 0;
		virtual void setHighTemperatureThreshold(float temperature) = 0;

		virtual int getSignalStrength() = 0;
		virtual float getDataRate() = 0;
		virtual bool isLinkHealthy() = 0;
		virtual std::string getLinkStatus() = 0;
	};

	class LogAndDiagnostics
	{
	public:
		virtual ~LogAndDiagnostics() = default;
		virtual std::string getErrorDescription(int errorCode, const std::string &language = "zh") = 0;
		virtual std::string getErrorSolution(int errorCode, const std::string &language = "zh") = 0;
		virtual std::vector<int> getErrorHistory() = 0;
		virtual std::map<int, int> getErrorStatistics() = 0;
		virtual bool performSystemCheck() = 0;
		virtual std::map<std::string, float> getSystemPerformance() = 0;
		virtual std::string generateHealthReport() = 0;
		virtual float getSystemHealthScore() = 0;
		virtual bool exportFlightLog(const std::string &filePath) = 0;
		virtual bool exportSystemLog(const std::string &filePath) = 0;
		virtual bool uploadLogsToCloud() = 0;
		virtual std::vector<std::string> getAvailableLogs() = 0;
		virtual bool clearLogs() = 0;
		virtual void setLogLevel(int level) = 0;
		virtual void enableRemoteLogging(bool enable) = 0;
	};

	/**
	 * @brief 视频源类型枚举
	 *
	 * 定义无人机支持的不同视频源类型
	 */
	enum class VideoSourceType
	{
		FPV,	///< FPV摄像头 - 第一人称视角摄像头，通常用于飞行导航
		GIMBAL ///< 云台摄像头 - 可控制的云台摄像头，支持多镜头切换 
	};

	/**
	 * @brief 视频源信息结构
	 *
	 * 包含视频源的详细信息和配置参数
	 */
	struct VideoSourceInfo
	{
		VideoSourceType type; ///< 视频源类型
		int index;			  ///< 云台镜头索引
	};

	/**
	 * @brief 视频传输控制类
	 *
	 * 提供无人机视频流传输、录制、截图等功能，支持多视频源切换
	 */
	class VideoTransmission
	{
	public:
		virtual ~VideoTransmission() = default;

		// ================================
		// 视频源切换相关接口
		// ================================
		
		/**
		 * @brief 切换视频源
		 * @param sourceInfo 目标视频源信息
		 * @return true 切换成功，false 切换失败
		 * @note ✅ 已实现
		 */
		virtual bool switchVideoSource(const VideoSourceInfo &sourceInfo) = 0;

		/**
		 * @brief 启动视频流
		 * @param sourceInfo 视频源信息
		 * @return true 启动成功，false 启动失败
		 * @note ❌ 未实现
		 */
		virtual bool startVideoStream(const VideoSourceInfo &sourceInfo) = 0;

		/**
		 * @brief 停止视频流
		 * @param sourceInfo 视频源信息
		 * @return true 停止成功，false 停止失败
		 * @note ❌ 未实现
		 */
		virtual bool stopVideoStream(const VideoSourceInfo &sourceInfo) = 0;

		/**
		 * @brief 获取视频流信息
		 * @param sourceInfo 视频源信息
		 * @return VideoStreamInfo 视频流信息
		 * @note ❌ 未实现
		 */
		virtual VideoStreamInfo getVideoStreamInfo(const VideoSourceInfo &sourceInfo) = 0;

		/**
		 * @brief 设置视频质量
		 * @param sourceInfo 视频源信息
		 * @param width 视频宽度
		 * @param height 视频高度
		 * @param framerate 帧率
		 * @return true 设置成功，false 设置失败
		 * @note ❌ 未实现
		 */
		virtual bool setVideoQuality(const VideoSourceInfo &sourceInfo, int width, int height, int framerate) = 0;

		/**
		 * @brief 启用/禁用运动检测
		 * @param enable true 启用，false 禁用
		 * @return true 设置成功，false 设置失败
		 * @note ❌ 未实现
		 */
		virtual bool enableMotionDetection(bool enable) = 0;

		/**
		 * @brief 启用/禁用目标跟踪
		 * @param enable true 启用，false 禁用
		 * @return true 设置成功，false 设置失败
		 * @note ❌ 未实现
		 */
		virtual bool enableTargetTracking(bool enable) = 0;

		/**
		 * @brief 设置跟踪目标位置
		 * @param x 目标X坐标
		 * @param y 目标Y坐标
		 * @return true 设置成功，false 设置失败
		 * @note ❌ 未实现
		 */
		virtual bool setTrackingTarget(int x, int y) = 0;

		/**
		 * @brief 获取检测到的运动目标
		 * @return std::vector<std::pair<int, int>> 运动目标坐标列表
		 * @note ❌ 未实现
		 */
		virtual std::vector<std::pair<int, int>> getDetectedMotion() = 0;

		/**
		 * @brief 设置视频流协议
		 * @param protocol 视频协议类型
		 * @return true 设置成功，false 设置失败
		 * @note ❌ 未实现
		 */
		virtual bool setStreamProtocol(VideoProtocol protocol) = 0;

		/**
		 * @brief 设置视频流URL
		 * @param url 视频流地址
		 * @return true 设置成功，false 设置失败
		 * @note ❌ 未实现
		 */
		virtual bool setStreamURL(const std::string &url) = 0;

		/**
		 * @brief 开始本地录制
		 * @param filePath 录制文件路径
		 * @return true 开始成功，false 开始失败
		 * @note ❌ 未实现
		 */
		virtual bool startLocalRecording(const std::string &filePath) = 0;

		/**
		 * @brief 停止本地录制
		 * @return true 停止成功，false 停止失败
		 * @note ❌ 未实现
		 */
		virtual bool stopLocalRecording() = 0;

		/**
		 * @brief 截取视频快照
		 * @param filePath 快照保存路径
		 * @return true 截图成功，false 截图失败
		 * @note ❌ 未实现
		 */
		virtual bool takeSnapshot(const std::string &filePath) = 0;

		 
	};

	class SDKCore
	{
	public:
		virtual ~SDKCore() = default;
		virtual std::shared_ptr<SDKManager> getSDKManager() = 0;
		virtual std::shared_ptr<FlightController> getFlightController() = 0;
		virtual std::shared_ptr<PayloadController> getPayloadController() = 0;
		virtual std::shared_ptr<SensorDataManager> getSensorDataManager() = 0;
		virtual std::shared_ptr<CloudInterface> getCloudInterface() = 0;
		virtual std::shared_ptr<BatteryAndLinkManager> getBatteryAndLinkManager() = 0;
		virtual std::shared_ptr<LogAndDiagnostics> getLogAndDiagnostics() = 0;
		virtual std::shared_ptr<VideoTransmission> getVideoTransmission() = 0;
		virtual std::shared_ptr<FlightParameterManager> getFlightParameterController() = 0;
		virtual bool initialize(const std::string &configPath) = 0;
		virtual bool initializeWithConfig(const SDKConfig &config) = 0;
		virtual bool shutdown() = 0;
		virtual std::string getLastError() const = 0;
		static std::unique_ptr<SDKCore> createInstance();
		static std::unique_ptr<SDKCore> createInstance(const SDKConfig &config);
	};

	/// Todo.....
	class ToolUtils
	{
	public:
		static Position3D convertToWGS84(const Position3D &localPos);
	};

	// ================================
	// EventListener 定义
	// ================================

	class EventListener
	{
	public:
		virtual void onMessageReceived(const nlohmann::json &message) = 0;
		virtual void onSensorDataReceived(const SensorData &sensorData) = 0;
		virtual ~EventListener() = default;
	};

} // namespace DroneSDK

#endif // __ZIYAN_API__