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

using StatusCallback = std::function<void(int status, const std::string& message)>;
using SensorDataCallback = std::function<void(const SensorData& data)>;  // 重命名避免冲突
using ErrorCallback = std::function<void(int errorCode, const std::string& errorMsg)>;

// ================================
// 前向声明
// ================================

namespace DroneSDK {
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
class SDKManager {
public:
	virtual ~SDKManager() = default;
	virtual std::string getSDKVersion() const = 0;
	virtual std::string getCompatibilityInfo() const = 0;
	virtual std::string getUpdateLog() const = 0;
	virtual bool initialize(const std::string& config) = 0;
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

class FlightController {
public:
	virtual ~FlightController() = default;
	virtual bool arm() = 0;
	virtual bool takeOff(float height) = 0;
	virtual bool guideToPosition(double latitude, double longitude, float altitude) = 0;
	virtual bool returnToHome() = 0;
	virtual bool land() = 0;
	virtual bool uploadMission(const Mission& mission,  const std::function<void(int current, int total)>& userProgressCallback) = 0;
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
	virtual bool setGeofence(const std::vector<Position3D>& polygon) = 0;
	virtual bool setCircularGeofence(const Position3D& center, float radius) = 0;
	virtual bool removeGeofence() = 0;
	virtual bool setFlightMode(FlightModeControl mode) = 0;
	virtual FlightModeControl getCurrentFlightMode() const = 0;

};


class PayloadController {
public:
	virtual ~PayloadController() = default;
	virtual bool setGimbalAttitude(float pitch, float yaw) = 0; 
	virtual bool calibrateGimbal() = 0;
	virtual bool resetGimbal() = 0;
	virtual bool opticalZoom(float length) = 0;
	virtual bool opticalSpeedZoom(float speed)=0;
	virtual bool digitalZoom(float zoom)=0;
	virtual bool takePhoto() = 0;
	virtual bool startRecording() = 0;
	virtual bool stopRecording() = 0;
	virtual void getCustomWidgetData(std::function<void(const json&)> callback) = 0;
	virtual bool setCustomWidgetValue(const uint8_t index, const std::string &widget_index, const uint8_t type,
								 const uint8_t value) = 0;
	virtual void subscribeGimbalAttitude( std::function<void(const json&)> callback) = 0;
};

class SensorDataManager {
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

class CloudInterface {
public:
	virtual ~CloudInterface() = default;
	virtual bool uploadTelemetryData(const SensorData& data) = 0;
	virtual bool enableRealTimeUpload(bool enable) = 0;
	virtual bool sendCommandFromCloud(const std::string& command, const std::vector<uint8_t>& params) = 0;
	virtual bool syncDeviceStatus() = 0;
	virtual bool startVideoStream(const std::string& streamKey, VideoProtocol protocol) = 0;
	virtual bool stopVideoStream() = 0;
	virtual VideoStreamInfo getVideoStreamInfo() const = 0;
	virtual bool setStreamQuality(int bitrate, int framerate) = 0;
	virtual bool connectToCloud(const std::string& endpoint, const std::string& token) = 0;
	virtual bool disconnectFromCloud() = 0;
	virtual bool isCloudConnectionStatus() const = 0;
	virtual std::string getCloudStatus() const = 0;
	virtual bool setupWebSocketConnection(const std::string& url) = 0;
	virtual bool sendWebSocketMessage(const std::string& message) = 0;
	virtual void setWebSocketCallback(std::function<void(const std::string&)> callback) = 0;
};

class BatteryAndLinkManager {
public:
	virtual ~BatteryAndLinkManager() = default;
	virtual BatteryInfo getBatteryInfo() = 0;
	virtual float getBatterySOH() = 0;  // State of Health
	virtual float getBatterySOC() = 0;  // State of Charge
	virtual bool isBatteryOverheated() = 0;
	virtual bool isLowBattery() = 0;
	virtual std::vector<float> getCellVoltages() = 0;
	virtual void setBatteryWarningCallback(std::function<void(const std::string&)> callback) = 0;
	virtual void setLowBatteryThreshold(float percentage) = 0;
	virtual void setHighTemperatureThreshold(float temperature) = 0;

	virtual int getSignalStrength() = 0;
	virtual float getDataRate() = 0;
	virtual bool isLinkHealthy() = 0;
	virtual std::string getLinkStatus() = 0;
};


class LogAndDiagnostics {
public:
    virtual ~LogAndDiagnostics() = default;
    virtual std::string getErrorDescription(int errorCode, const std::string& language = "zh") = 0;
    virtual std::string getErrorSolution(int errorCode, const std::string& language = "zh") = 0;
    virtual std::vector<int> getErrorHistory() = 0;
    virtual std::map<int, int> getErrorStatistics() = 0;
    virtual bool performSystemCheck() = 0;
    virtual std::map<std::string, float> getSystemPerformance() = 0;
    virtual std::string generateHealthReport() = 0;
    virtual float getSystemHealthScore() = 0;
    virtual bool exportFlightLog(const std::string& filePath) = 0;
    virtual bool exportSystemLog(const std::string& filePath) = 0;
    virtual bool uploadLogsToCloud() = 0;
    virtual std::vector<std::string> getAvailableLogs() = 0;
    virtual bool clearLogs() = 0;
    virtual void setLogLevel(int level) = 0;
    virtual void enableRemoteLogging(bool enable) = 0;
};

class VideoTransmission {
public:
    virtual ~VideoTransmission() = default;
    virtual bool startMainCameraStream() = 0;
    virtual bool stopMainCameraStream() = 0;
    virtual VideoStreamInfo getMainCameraStreamInfo() = 0;
    virtual bool setMainCameraQuality(int width, int height, int framerate) = 0;
    virtual bool startFPVStream() = 0;
    virtual bool stopFPVStream() = 0;
    virtual VideoStreamInfo getFPVStreamInfo() = 0;
    virtual bool enableMotionDetection(bool enable) = 0;
    virtual bool enableTargetTracking(bool enable) = 0;
    virtual bool setTrackingTarget(int x, int y) = 0;
    virtual std::vector<std::pair<int, int>> getDetectedMotion() = 0;
    virtual bool setStreamProtocol(VideoProtocol protocol) = 0;
    virtual bool setStreamURL(const std::string& url) = 0;
    virtual bool startLocalRecording(const std::string& filePath) = 0;
    virtual bool stopLocalRecording() = 0;
    virtual bool takeSnapshot(const std::string& filePath) = 0;
};

class SDKCore {
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
    virtual bool initialize(const std::string& configPath) = 0;
    virtual bool initializeWithConfig(const SDKConfig& config) = 0;
    virtual bool shutdown() = 0;
    virtual std::string getLastError() const = 0;
    static std::unique_ptr<SDKCore> createInstance();
    static std::unique_ptr<SDKCore> createInstance(const SDKConfig& config);
};

///Todo.....
class ToolUtils {
public:
    static Position3D convertToWGS84(const Position3D& localPos);
};

// ================================
// EventListener 定义
// ================================

class EventListener {
public:
    virtual void onMessageReceived(const nlohmann::json& message) = 0;
    virtual void onSensorDataReceived(const SensorData& sensorData) = 0; 
    virtual ~EventListener() = default;
};

} // namespace DroneSDK

#endif // __ZIYAN_API__