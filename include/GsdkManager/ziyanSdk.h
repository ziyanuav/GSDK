#ifndef __ZIYAN_SDK_H__
#define __ZIYAN_SDK_H__

#include <iostream>
#include <thread>
#include <mutex>
#include <memory>
#include <map>
#include <vector>
#include <nlohmann/json.hpp>

using namespace std;

namespace DroneSDK
{
    class SDKManager;
    class FlightController;
    class PayloadController;
    class SensorDataManager;
    class CloudInterface;
    class BatteryAndLinkManager;
    class LogAndDiagnostics;
    class VideoTransmission;
    class FlightParameterManager;
    class SDKCore;
    struct SDKConfig;

    // 飞行参数类型枚举 - 放在SDK公共接口中
    enum class FlightParameterType
    {
        // 返航与安全参数
        RTL_ALT,         // 返航高度 (cm)
        RTL_ALT_FINAL,   // 最终返航悬停高度 (cm)
        FS_GCS_ENABLE,   // 地面站通信丢失保护使能
        BATT_FS_LOW_ACT, // 主电池低电量动作
        BATT_LOW_VOLT,   // 主电池低电压阈值 (V)
        AVOID_ENABLE,    // 避障功能使能
        AVOID_MARGIN,    // 避障安全距离 (m)
        WPNAV_SPEED,     // 航点水平巡航速度 (m/s)
        WPNAV_SPEED_UP,  // 航点上升速度 (m/s)
        WPNAV_SPEED_DN,  // 航点下降速度 (m/s)
        MAG_ENABLE,      // 磁力计使能
        COMPASS_CAL_FIT, // 磁力计校准适合度
        UNKNOWN
    };
    enum PointCommand
    {
        LOITER = 19, // 悬停  p1 悬停时长秒
        WAYPOINT = 16, // 普通航点
        SURVEY = 13, // 测绘航点 p1 是否开启拍照1开0关,p2 等距离(单位米)/等时间（单位s），p4 0距离类型1时间类形
        WAYPOINT_JUMP = 177, // 航点跳转 p1跳转航点,p2跳转次数
        SPEED_CHANGE = 178, // 飞行速度变更 p2速度 米每秒
        MOUNT_CONTROL = 205, // 云台控制 p1 俯仰角度，p3 偏移角
        DIGICAM_SHOT = 203, // 拍照
        TAKEOFF = 22, // 起飞
        RETURN_TO_LAUNCH = 20, // 返航
        LAND = 21, // 原地降落
        CONDITION_YAW = 115 ,// 改变航向
        LOITER_TURNS=18//周圆飞行  p1 周期数  p3 半径
    };
    // 参数值类型
    enum class ParameterValueType
    {
        FLOAT, // 浮点型
        INT,   // 整型
        BOOL,  // 布尔型
        STRING // 字符串型
    };

    // 参数信息结构
    struct FlightParameterInfo
    {
        std::string paramId;          // 参数ID
        std::string description;      // 参数描述
        ParameterValueType valueType; // 参数值类型
        float minValue;               // 最小值
        float maxValue;               // 最大值
        float defaultValue;           // 默认值
        float currentValue;           // 当前值
        bool isReadOnly;              // 是否只读
        std::string unit;             // 单位
    };

    // 参数状态结构
    struct ParameterStatus
    {
        std::string paramId;
        float value;
        bool success;
        std::string errorMessage;
        std::chrono::steady_clock::time_point timestamp;
    };

    // 回调函数类型定义
    using ParameterReadCallback = std::function<void(const ParameterStatus &)>;
    using ParameterWriteCallback = std::function<void(const ParameterStatus &)>;
    using ParameterChangeCallback = std::function<void(FlightParameterType, float)>;
}

struct Position3D;
struct Mission;
struct Attitude;
struct BatteryInfo;
struct SensorData;
struct FlightState;
struct VideoStreamInfo;
namespace ErrorCodes
{
    const int SUCCESS = 0;
    const int INIT_FAILED = -1001;
    const int CONNECTION_LOST = -1002;
    const int INVALID_PARAMETER = -1003;
    const int OPERATION_TIMEOUT = -1004;
    const int BATTERY_LOW = -1005;
    const int GPS_SIGNAL_WEAK = -1006;
    const int OBSTACLE_DETECTED = -1007;
    const int GEOFENCE_VIOLATION = -1008;
    const int MISSION_FAILED = -1009;
    const int PAYLOAD_ERROR = -1010;
    const int STREAM_ERROR = -1011;
    const int CLOUD_UPLOAD_FAILED = -1012;
}
enum class FlightModeControl
{
    MANUAL,
    AUTO_MISSION,
    RTH,
    HOVER
};

enum class VideoProtocol
{
    RTMP,
    RTSP,
    WEBSOCKET
};

struct Position3D
{
    double latitude;
    double longitude;
    double altitude;
    double relativeAltitude;
};

struct Attitude
{
    float pitch;
    float roll;
    float yaw;
};

struct Velocity3D
{
    float vx;
    float vy;
    float vz;
};

struct BatteryInfo
{
    float voltage;
    float current;
    int percentage;
    int cellCount;
};

typedef enum
{
    SENSOR_POSITION,         // 位置数据
    SENSOR_ATTITUDE,         // 姿态数据
    SENSOR_VELOCITY,         // 速度数据
    SENSOR_HIGH_FREQUENCY,   // 高频传感器数据
    SENSOR_BATTERY,          // 电池数据
    SENSOR_LOW_FREQUENCY,    // 低频状态数据
    SENSOR_EXTRA,            // 扩展数据
    SENSOR_VIDEO_ENCODER,    // 视频编码数据 
} SensorType;

enum class VideoEncoderType
{
    H264,
    H265,
    MPEG4
};

enum class VideoNaluType
{
    SPS,
    PPS,
    IDR,
    NON_IDR,
    SEI
};

struct VideoEncoderData
{
    VideoEncoderType encoderType;
    uint32_t size;
    char *buffer;
    int64_t timestamp;
    VideoNaluType naluType;
};

struct HighFrequencyData
{
    float imu1_accel_norm; // IMU1加速度模值
    float imu1_gyro_norm;  // IMU1陀螺仪模值
    float imu1_mag_norm;   // IMU1磁力计模值
    float imu2_accel_norm; // IMU2加速度模值
    float imu2_gyro_norm;  // IMU2陀螺仪模值
    float imu2_mag_norm;   // IMU2磁力计模值
    float imu3_accel_norm; // IMU3加速度模值
    float imu3_gyro_norm;  // IMU3陀螺仪模值
    float imu3_mag_norm;   // IMU3磁力计模值

    double gimbal_roll;  // 云台滚转角
    double gimbal_pitch; // 云台俯仰角
    double gimbal_yaw;   // 云台偏航角

    float front_distance;   // 前方避障距离
    float horizontal_speed; // 水平速度
    float vertical_speed;   // 垂直速度
    int satellites_visible; // 可见卫星数

    float main_rpm;     // 主桨转速
    float tail_rpm;     // 尾桨转速
    float main_power;   // 主浆功率
    float tail_power;   // 尾浆功率
    float main_current; // 主浆电流
    float tail_current; // 尾浆电流
    float main_temp;    // 主浆电调温度
    float tail_temp;    // 尾浆电调温度
    float raw_temp;     // 电调温度
};

struct LowFrequencyData
{
    double home_latitude;  // Home点纬度
    double home_longitude; // Home点经度
    float home_altitude;   // Home点高度
    float home_distance;   // 距离Home点距离

    int signal_strength;     // 信号强度
    bool front_avoid_switch; // 前避障开关
    bool armed;              // 是否解锁
    bool landed_state;       // 是否在地面
    uint32_t flight_mode;    // 飞行模式
    int fly_time;            // 飞行时长
    double fly_distance;     // 飞行距离
};

struct ExtraData
{
    HighFrequencyData high_freq;
    LowFrequencyData low_freq;

    ExtraData() = default;
    double get_home_latitude() const { return low_freq.home_latitude; }
    double get_home_longitude() const { return low_freq.home_longitude; }
    float get_home_altitude() const { return low_freq.home_altitude; }
    float get_home_distance() const { return low_freq.home_distance; }

    double get_gimbal_roll() const { return high_freq.gimbal_roll; }
    double get_gimbal_pitch() const { return high_freq.gimbal_pitch; }
    double get_gimbal_yaw() const { return high_freq.gimbal_yaw; }

    float get_imu1_accel_norm() const { return high_freq.imu1_accel_norm; }
    float get_imu1_gyro_norm() const { return high_freq.imu1_gyro_norm; }
    float get_imu1_mag_norm() const { return high_freq.imu1_mag_norm; }
    float get_imu2_accel_norm() const { return high_freq.imu2_accel_norm; }
    float get_imu2_gyro_norm() const { return high_freq.imu2_gyro_norm; }
    float get_imu2_mag_norm() const { return high_freq.imu2_mag_norm; }
    float get_imu3_accel_norm() const { return high_freq.imu3_accel_norm; }
    float get_imu3_gyro_norm() const { return high_freq.imu3_gyro_norm; }
    float get_imu3_mag_norm() const { return high_freq.imu3_mag_norm; }

    int get_signal_strength() const { return low_freq.signal_strength; }
    int get_satellites_visible() const { return high_freq.satellites_visible; }
    float get_front_distance() const { return high_freq.front_distance; }
    bool get_front_avoid_switch() const { return low_freq.front_avoid_switch; }
    float get_horizontal_speed() const { return high_freq.horizontal_speed; }
    float get_vertical_speed() const { return high_freq.vertical_speed; }

    bool get_armed() const { return low_freq.armed; }
    bool get_landed_state() const { return low_freq.landed_state; }
    uint32_t get_flight_mode() const { return low_freq.flight_mode; }
    int get_fly_time() const { return low_freq.fly_time; }
    double get_fly_distance() const { return low_freq.fly_distance; }

    // 直接访问方式
    double &ref_home_latitude() { return low_freq.home_latitude; }
    double &ref_home_longitude() { return low_freq.home_longitude; }
    float &ref_home_altitude() { return low_freq.home_altitude; }
    float &ref_home_distance() { return low_freq.home_distance; }

    double &ref_gimbal_roll() { return high_freq.gimbal_roll; }
    double &ref_gimbal_pitch() { return high_freq.gimbal_pitch; }
    double &ref_gimbal_yaw() { return high_freq.gimbal_yaw; }

    float &ref_imu1_accel_norm() { return high_freq.imu1_accel_norm; }
    float &ref_imu1_gyro_norm() { return high_freq.imu1_gyro_norm; }
    float &ref_imu1_mag_norm() { return high_freq.imu1_mag_norm; }
    int &ref_signal_strength() { return low_freq.signal_strength; }
    int &ref_satellites_visible() { return high_freq.satellites_visible; }
    float &ref_front_distance() { return high_freq.front_distance; }
    bool &ref_front_avoid_switch() { return low_freq.front_avoid_switch; }
    float &ref_horizontal_speed() { return high_freq.horizontal_speed; }
    float &ref_vertical_speed() { return high_freq.vertical_speed; }

    bool &ref_armed() { return low_freq.armed; }
    bool &ref_landed_state() { return low_freq.landed_state; }
    uint32_t &ref_flight_mode() { return low_freq.flight_mode; }
    int &ref_fly_time() { return low_freq.fly_time; }
    double &ref_fly_distance() { return low_freq.fly_distance; }
};

// 基于实际Vehicle数据结构的传感器数据
struct SensorData
{
    SensorType type;
    std::chrono::steady_clock::time_point timestamp; // 数据时间戳

    union
    {
        struct Position3D position;        // 位置数据（纬度、经度、海拔）
        struct Attitude attitude;          // 姿态数据（roll, pitch, yaw）
        struct Velocity3D velocity;        // 速度数据（vx, vy, vz）
        struct BatteryInfo battery;        // 电池数据（电压、电流、百分比）
        struct VideoEncoderData videoData; // 视频编码数据

        // 新增的高频和低频数据类型
        struct HighFrequencyData highFreq; // 高频传感器数据
        struct LowFrequencyData lowFreq;   // 低频状态数据

        // 兼容性数据类型
        struct ExtraData extra; // 扩展数据（向后兼容）
    } data;

    // 构造函数
    SensorData() : type(SENSOR_POSITION), timestamp(std::chrono::steady_clock::now())
    {
        new (&data.position) Position3D{0.0, 0.0, 0.0, 0.0};
    }

    // 添加析构函数
    ~SensorData()
    {
        // union不需要显式析构
    }

    // 拷贝构造函数
    SensorData(const SensorData &other) : type(other.type), timestamp(other.timestamp)
    {
        switch (type)
        {
        case SENSOR_POSITION:
            new (&data.position) Position3D(other.data.position);
            break;
        case SENSOR_ATTITUDE:
            new (&data.attitude) Attitude(other.data.attitude);
            break;
        case SENSOR_VELOCITY:
            new (&data.velocity) Velocity3D(other.data.velocity);
            break;
        case SENSOR_BATTERY:
            new (&data.battery) BatteryInfo(other.data.battery);
            break;
        case SENSOR_VIDEO_ENCODER:
            new (&data.videoData) VideoEncoderData(other.data.videoData);
            break;
        case SENSOR_HIGH_FREQUENCY:
            new (&data.highFreq) HighFrequencyData(other.data.highFreq);
            break;
        case SENSOR_LOW_FREQUENCY:
            new (&data.lowFreq) LowFrequencyData(other.data.lowFreq);
            break;
        case SENSOR_EXTRA:
            new (&data.extra) ExtraData(other.data.extra);
            break;
        default:
            new (&data.position) Position3D{0.0, 0.0, 0.0, 0.0};
            break;
        }
    }
};

struct MissionPoint
{
    Position3D position;
    float speed;
    DroneSDK::PointCommand actionType;
    std::map<std::string, float> params;
};

struct Mission
{
    std::vector<MissionPoint> waypoints;
    float cruiseSpeed;
    float finishedAction;
    bool exitMissionOnRCSignalLost;
};

struct FlightState
{
    bool isFlying;
    bool isLanded;
    bool isHomePointSet;
    FlightModeControl currentMode;
    Position3D homePoint;
};

struct VideoStreamInfo
{
    std::string streamUrl;
    int width;
    int height;
    int framerate;
    int bitrate;
};

namespace Constants
{
    const float MAX_ALTITUDE = 500.0f;          // �����и߶�(��)
    const float MAX_DISTANCE = 8000.0f;         // �����о���(��)
    const float MIN_BATTERY_PERCENTAGE = 20.0f; // ��͵�ص����ٷֱ�
    const float MAX_WIND_SPEED = 12.0f;         // ��󿹷�ȼ�(m/s)
    const int MAX_WAYPOINTS = 99;               // ��󺽵�����?
    const int DEFAULT_DATA_FREQUENCY = 5;       // Ĭ����������Ƶ��(Hz)
    const int VIDEO_STREAM_TIMEOUT = 30;        // ��Ƶ����ʱʱ��(��)
}

class EventListener
{
public:
    // ================================
    // 定频数据回调（周期性推送，10Hz~50Hz）
    // ================================
    virtual void onSensorDataReceived(const SensorData &sensorData) = 0;

    // ================================
    // 状态/属性数据回调（事件驱动推送）
    // ================================

    // 连接状态事件
    virtual void onConnectionStatusChanged(bool connected, const std::string &connectionType = "") = 0;

    // 飞行状态事件
    // virtual void onFlightStatusChanged(bool isFlying) = 0;
    virtual void onArmStatusChanged(bool armed) = 0;

    // 设备信息事件（连接时推送一次）
    // product_id  Unknow = 0,S3 = 1, F15 = 2, GWG1 = 3
    virtual void onDeviceInfoUpdated(const std::string &sn, const int32_t &product_id) = 0;

    // 避障状态事件
    virtual void onObstacleAvoidanceChanged(float frontDistance, bool frontSwitch) = 0;

    // ================================
    // 通用消息回调
    // ================================
    virtual void onMessageReceived(const nlohmann::json &message) = 0;

    virtual ~EventListener() = default;
};

namespace DroneSDK
{

    struct SDKConfig
    {
        std::string deviceType;
        std::string serialPort;
        std::string productKey;
        std::string logLevel;
        int baudRate;
        int timeoutMs;
        bool enableLogging;
        std::string logPath;
        bool autoConnect;
        std::string cloudEndpoint;
        std::string cloudToken;
        std::shared_ptr<::EventListener> eventLoopHandler;
    };
    class ZiyanSdk
    {
    public:
        ZiyanSdk();
        ~ZiyanSdk() = default;
        bool initialize(const std::string &configPath);
        bool initializeWithConfig(const SDKConfig &config);
        bool shutdown();
        std::shared_ptr<SDKManager> getSDKManager();
        std::shared_ptr<FlightController> getFlightController();
        std::shared_ptr<PayloadController> getPayloadController();
        std::shared_ptr<SensorDataManager> getSensorDataManager();
        std::shared_ptr<CloudInterface> getCloudInterface();
        std::shared_ptr<BatteryAndLinkManager> getBatteryAndLinkManager();
        std::shared_ptr<LogAndDiagnostics> getLogAndDiagnostics();
        std::shared_ptr<VideoTransmission> getVideoTransmission();
        std::shared_ptr<FlightParameterManager> getFlightParameterController();
        std::string getLastError() const;
        std::string getDeviceSerialNumber() const;

    private:
        std::unique_ptr<SDKCore> sdkCore;
    };

} // namespace DroneSDK

#endif // __ZIYAN_SDK_H__
