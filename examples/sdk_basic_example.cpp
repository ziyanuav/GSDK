#include <nlohmann/json.hpp>
#include "GsdkManager/ziyanSdk.h"
#include "GsdkManager/sdkInterface.h"
#include <iostream>
#include <thread>

using namespace std;

class ExampleListener : public EventListener {
public:
      void onSensorDataReceived(const SensorData &sensorData) override
    {
        switch (sensorData.type)
        {
        case SENSOR_POSITION:
            std::cout << "位置数据: 纬度=" << std::fixed << std::setprecision(6)
                      << sensorData.data.position.latitude << " 经度=" << sensorData.data.position.longitude
                      << " 海拔=" << std::setprecision(1) << sensorData.data.position.altitude << "m" << std::endl;
            break;

        case SENSOR_ATTITUDE:
            std::cout << "姿态数据: 俯仰角=" << std::fixed << std::setprecision(1)
                      << sensorData.data.attitude.pitch << "° 滚转角=" << sensorData.data.attitude.roll
                      << "° 偏航角=" << sensorData.data.attitude.yaw << "°" << std::endl;
            break;

        case SENSOR_VELOCITY:
            std::cout << "速度数据: X轴=" << std::fixed << std::setprecision(1)
                      << sensorData.data.velocity.vx << "m/s Y轴=" << sensorData.data.velocity.vy
                      << "m/s Z轴=" << sensorData.data.velocity.vz << "m/s" << std::endl;
            break;

        case SENSOR_BATTERY:
            std::cout << "电池数据: 电量=" << sensorData.data.battery.percentage << "% 电压="
                      << sensorData.data.battery.voltage << "V 电流=" << sensorData.data.battery.current << "A" << std::endl;
            break;

        case SENSOR_HIGH_FREQUENCY:
        {
            const auto &hf = sensorData.data.highFreq;

            std::cout << "\n=== 高频传感器数据 ===" << std::endl;
            std::cout << "IMU1: 加速度=" << hf.imu1_accel_norm << "m/s² 陀螺仪=" << hf.imu1_gyro_norm
                      << "rad/s 磁力计=" << hf.imu1_mag_norm << "μT" << std::endl;
            std::cout << "IMU2: 加速度=" << hf.imu2_accel_norm << "m/s² 陀螺仪=" << hf.imu2_gyro_norm
                      << "rad/s 磁力计=" << hf.imu2_mag_norm << "μT" << std::endl;
            std::cout << "IMU3: 加速度=" << hf.imu3_accel_norm << "m/s² 陀螺仪=" << hf.imu3_gyro_norm
                      << "rad/s 磁力计=" << hf.imu3_mag_norm << "μT" << std::endl;

            std::cout << "云台姿态: 滚转=" << std::fixed << std::setprecision(1) << hf.gimbal_roll
                      << "° 俯仰=" << hf.gimbal_pitch << "° 偏航=" << hf.gimbal_yaw << "°" << std::endl;
            std::cout << "前方避障距离: " << hf.front_distance << "m" << std::endl;
            std::cout << "水平速度: " << hf.horizontal_speed << "m/s 垂直速度: " << hf.vertical_speed << "m/s" << std::endl;
            std::cout << "可见卫星数: " << hf.satellites_visible << "颗" << std::endl;
            std::cout << "主桨转速: " << hf.main_rpm << "RPM 尾桨转速: " << hf.tail_rpm << "RPM" << std::endl;
            std::cout << "主浆功率: " << hf.main_power << "W 尾浆功率: " << hf.tail_power << "W" << std::endl;
            std::cout << "主浆电流: " << hf.main_current << "A 尾浆电流: " << hf.tail_current << "A" << std::endl;
            std::cout << "主浆温度: " << hf.main_temp << "℃ 尾浆温度: " << hf.tail_temp << "℃" << std::endl;
            std::cout << "电调温度: " << hf.raw_temp << "℃" << std::endl;

            break;
        }

        case SENSOR_LOW_FREQUENCY:
        {
            const auto &lf = sensorData.data.lowFreq;

            std::cout << "\n=== 低频状态数据 ===" << std::endl;
            std::cout << "Home点坐标: " << std::fixed << std::setprecision(6)
                      << lf.home_latitude << ", " << lf.home_longitude
                      << " 高度: " << std::setprecision(1) << lf.home_altitude << "m" << std::endl;
            std::cout << "距离Home点: " << lf.home_distance << "m" << std::endl;



            std::cout << "信号强度: " << lf.signal_strength << "/5" << std::endl;
            std::cout << "避障开关: " << (lf.front_avoid_switch ? "开启" : "关闭") << std::endl;
            std::cout << "解锁状态: " << (lf.armed ? "已解锁" : "未解锁") << std::endl;
            std::cout << "着陆状态: " << (lf.landed_state ? "在地面" : "飞行中") << std::endl;
            std::cout << "飞行模式: " << lf.flight_mode << std::endl;
            std::cout << "飞行时长: " << lf.fly_time << "秒" << std::endl;
            std::cout << "飞行距离: " << std::fixed << std::setprecision(1) << lf.fly_distance << "米" << std::endl;

            break;
        }

        case SENSOR_EXTRA:
        {
            const auto &ex = sensorData.data.extra;

            std::cout << "Home点坐标: " << std::fixed << std::setprecision(6)
                      << ex.get_home_latitude() << ", " << ex.get_home_longitude()
                      << " 高度: " << std::setprecision(1) << ex.get_home_altitude() << "m" << std::endl;
            std::cout << "距离Home点: " << ex.get_home_distance() << "m" << std::endl;

            std::cout << "云台姿态: 滚转=" << ex.get_gimbal_roll()
                      << "° 俯仰=" << ex.get_gimbal_pitch() << "° 偏航=" << ex.get_gimbal_yaw() << "°" << std::endl;

            std::cout << "IMU1: 加速度=" << ex.get_imu1_accel_norm() << "m/s² 陀螺仪=" << ex.get_imu1_gyro_norm()
                      << "rad/s 磁力计=" << ex.get_imu1_mag_norm() << "μT" << std::endl;

            std::cout << "信号强度: " << ex.get_signal_strength() << "/5" << std::endl;
            std::cout << "前方避障距离: " << ex.get_front_distance() << "m" << std::endl;
            std::cout << "避障开关: " << (ex.get_front_avoid_switch() ? "开启" : "关闭") << std::endl;

            std::cout << "飞行状态: " << (ex.get_armed() ? "已解锁" : "未解锁") << " "
                      << (ex.get_landed_state() ? "在地面" : "飞行中") << " 模式:" << ex.get_flight_mode()
                      << " 时长:" << ex.get_fly_time() << "秒 距离:" << ex.get_fly_distance() << "m" << std::endl;

            break;
        }

        default:
            std::cout << "未知传感器数据类型: " << static_cast<int>(sensorData.type) << std::endl;
            break;
        }
    }

    void onConnectionStatusChanged(bool connected, const std::string &connectionType) override
    {
        std::cout << "\n=== 连接状态变更 ===" << std::endl;
        std::cout << "连接状态: " << (connected ? "已连接" : "已断开") << std::endl;
    }

    void onArmStatusChanged(bool armed) override
    {
        std::cout << "\n=== 解锁状态变更 ===" << std::endl;
        std::cout << "解锁状态: " << (armed ? "已解锁" : "已锁定") << std::endl;
    }

    void onDeviceInfoUpdated(const std::string &sn, const int32_t &product_id) override
    {
        std::cout << "\n=== 设备信息更新 ===" << std::endl;
        std::cout << "设备序列号: " << sn << std::endl;
    }

    void onObstacleAvoidanceChanged(float frontDistance, bool frontSwitch) override
    {
        std::cout << "\n=== 避障状态变更 ===" << std::endl;
        std::cout << "前方距离: " << frontDistance << "m" << std::endl;
        std::cout << "避障开关: " << (frontSwitch ? "开启" : "关闭") << std::endl;
    }

    void onMessageReceived(const nlohmann::json &message) override
    {
        std::cout << "收到消息: " << message.dump() << std::endl;
    }
};

int main() {
    DroneSDK::ZiyanSdk sdk;
    DroneSDK::SDKConfig config;
    config.logLevel = "info";
    config.enableLogging = true;
    config.logPath = "./logs";
    config.autoConnect = true;
    config.eventLoopHandler =  std::static_pointer_cast<EventListener>(std::make_shared<ExampleListener>());

    if (!sdk.initializeWithConfig(config)) {
        std::cerr << "SDK init failed: " << sdk.getLastError() << std::endl;
        return -1;
    }

    auto flight = sdk.getFlightController();
    if (flight) {
        std::cout << "Flight controller ready" << std::endl;
        flight->takeOff(20);
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));

    sdk.shutdown();
    std::cout << "SDK shutdown" << std::endl;
    return 0;
}

