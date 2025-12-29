#include <nlohmann/json.hpp>
#include "GsdkManager/ziyanSdk.h"
#include "GsdkManager/sdkInterface.h"
#include <iostream>
#include <thread>
#include <atomic>
#include <fstream>
#include <cstdint>

using namespace std;

std::atomic<bool> g_exitFlag(false);  // 程序退出标志

class SDKListener : public EventListener {
public:
    void onSensorDataReceived(const SensorData &sensorData) override
    {
        switch (sensorData.type)
        {
        case SENSOR_VIDEO_ENCODER:
            {
                const uint32_t bufferSize = sensorData.data.videoData.size;
                std::cout << "视频裸数据 bufferSize:" << bufferSize << std::endl;

                const VideoEncoderData& videoData = sensorData.data.videoData;
                static std::ofstream filewrite("swarm_dump_28_rtp.264", std::ios::binary);
                
                if (filewrite.is_open() && videoData.buffer != nullptr && videoData.size > 0)
                {
                    filewrite.write(reinterpret_cast<const char*>(videoData.buffer), videoData.size);
                    std::cout << "[SwarmCamera] receive videoData.buffer size:" << videoData.size << std::endl;

                     // ========== 核心：打印 buffer 为十六进制字符串 ==========
                    std::cout << "[SwarmCamera] videoData.buffer 十六进制内容: ";
                    // 转换为 uint8_t* 方便逐字节读取（H264 是字节流）
                    const uint8_t* bufferPtr = reinterpret_cast<const uint8_t*>(videoData.buffer);
                    // 控制打印长度：避免数据过大刷屏，默认打印前32字节（可自行调整）
                    const uint32_t printLen = std::min(bufferSize, 32U); 
                    for (uint32_t i = 0; i < printLen; ++i)
                    {
                        // 格式化：两位十六进制，不足补0，大写显示（符合裸流调试习惯）
                        std::cout << std::hex << std::uppercase << std::setfill('0') << std::setw(2) 
                                << static_cast<int>(bufferPtr[i]) << " ";
                    }
                    // 如果数据超过32字节，提示省略部分
                    if (bufferSize > printLen)
                    {
                        std::cout << "... (省略后续 " << (bufferSize - printLen) << " 字节)";
                    }
                    std::cout << std::dec << std::endl;  // 恢复十进制输出格式
                    // ======================================================

                }
                else if (!filewrite.is_open())
                {
                    std::cout << "[SwarmCamera] filewrite open failed!" << std::endl;
                }

                break;
            }
        case SENSOR_POSITION:
            {
                std::cout << "位置数据: 纬度=" << std::fixed << std::setprecision(6)
                      << sensorData.data.position.latitude << " 经度=" << sensorData.data.position.longitude
                      << " 海拔=" << std::setprecision(1) << sensorData.data.position.altitude << "m"
                      << " 相对高度" << std::setprecision(1) << sensorData.data.position.relativeAltitude << "m" << std::endl;

                break;
            }

        case SENSOR_ATTITUDE:
            {
                // std::cout << "姿态数据: 俯仰角=" << std::fixed << std::setprecision(1)
                //       << sensorData.data.attitude.pitch << "° 滚转角=" << sensorData.data.attitude.roll
                //       << "° 偏航角=" << sensorData.data.attitude.yaw << "°" << std::endl;
                break;
            }

        case SENSOR_VELOCITY:
            {
                // std::cout << "速度数据: X轴=" << std::fixed << std::setprecision(1)
                //       << sensorData.data.velocity.vx << "m/s Y轴=" << sensorData.data.velocity.vy
                //       << "m/s Z轴=" << sensorData.data.velocity.vz << "m/s" << std::endl;
                break;
            }

        case SENSOR_BATTERY:
            {
                // std::cout << "电池数据: 电量=" << sensorData.data.battery.percentage << "% 电压="
                //       << sensorData.data.battery.voltage << "V 电流=" << sensorData.data.battery.current << "A" << std::endl;
                break;
            }

        case SENSOR_BATTERIES:
        {
            const auto &arr = sensorData.data.batteries;
            std::cout << "电池数组(count=" << arr.count << "):" << std::endl;
            for (int i = 0; i < arr.count; ++i)
            {
                const auto &b = arr.items[i];
                std::cout << "  [" << i << "] voltage=" << b.voltage
                          << "V current=" << b.current
                          << "A remaining=" << b.percentage
                          << "% cell_count=" << b.cellCount << std::endl;
            }
            break;
        }


        case SENSOR_HIGH_FREQUENCY:
            {
                const auto &hf = sensorData.data.highFreq;

                // std::cout << "\n=== 高频传感器数据 ===" << std::endl;
                // std::cout << "IMU1: 加速度=" << hf.imu1_accel_norm << "m/s² 陀螺仪=" << hf.imu1_gyro_norm
                //         << "rad/s 磁力计=" << hf.imu1_mag_norm << "μT" << std::endl;
                // std::cout << "IMU2: 加速度=" << hf.imu2_accel_norm << "m/s² 陀螺仪=" << hf.imu2_gyro_norm
                //         << "rad/s 磁力计=" << hf.imu2_mag_norm << "μT" << std::endl;
                // std::cout << "IMU3: 加速度=" << hf.imu3_accel_norm << "m/s² 陀螺仪=" << hf.imu3_gyro_norm
                //         << "rad/s 磁力计=" << hf.imu3_mag_norm << "μT" << std::endl;

                // std::cout << "云台姿态: 滚转=" << std::fixed << std::setprecision(1) << hf.gimbal_roll
                //         << "° 俯仰=" << hf.gimbal_pitch << "° 偏航=" << hf.gimbal_yaw << "°" << std::endl;
                // std::cout << "前方避障距离: " << hf.front_distance << "m" << std::endl;
                // std::cout << "水平速度: " << hf.horizontal_speed << "m/s 垂直速度: " << hf.vertical_speed << "m/s" << std::endl;
                // std::cout << "可见卫星数: " << hf.satellites_visible << "颗" << std::endl;
                // std::cout << "主桨转速: " << hf.main_rpm << "RPM 尾桨转速: " << hf.tail_rpm << "RPM" << std::endl;
                // std::cout << "主浆功率: " << hf.main_power << "W 尾浆功率: " << hf.tail_power << "W" << std::endl;
                // std::cout << "主浆电流: " << hf.main_current << "A 尾浆电流: " << hf.tail_current << "A" << std::endl;
                // std::cout << "主浆温度: " << hf.main_temp << "℃ 尾浆温度: " << hf.tail_temp << "℃" << std::endl;
                // std::cout << "电调温度: " << hf.raw_temp << "℃" << std::endl;

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

            // std::cout << "Home点坐标: " << std::fixed << std::setprecision(6)
            //           << ex.get_home_latitude() << ", " << ex.get_home_longitude()
            //           << " 高度: " << std::setprecision(1) << ex.get_home_altitude() << "m" << std::endl;
            // std::cout << "距离Home点: " << ex.get_home_distance() << "m" << std::endl;

            // std::cout << "云台姿态: 滚转=" << ex.get_gimbal_roll()
            //           << "° 俯仰=" << ex.get_gimbal_pitch() << "° 偏航=" << ex.get_gimbal_yaw() << "°" << std::endl;

            // std::cout << "IMU1: 加速度=" << ex.get_imu1_accel_norm() << "m/s² 陀螺仪=" << ex.get_imu1_gyro_norm()
            //           << "rad/s 磁力计=" << ex.get_imu1_mag_norm() << "μT" << std::endl;

            // std::cout << "信号强度: " << ex.get_signal_strength() << "/5" << std::endl;
            // std::cout << "前方避障距离: " << ex.get_front_distance() << "m" << std::endl;
            // std::cout << "避障开关: " << (ex.get_front_avoid_switch() ? "开启" : "关闭") << std::endl;

            // std::cout << "飞行状态: " << (ex.get_armed() ? "已解锁" : "未解锁") << " "
            //           << (ex.get_landed_state() ? "在地面" : "飞行中") << " 模式:" << ex.get_flight_mode()
            //           << " 时长:" << ex.get_fly_time() << "秒 距离:" << ex.get_fly_distance() << "m" << std::endl;

            break;
        }

        default:
            {
                //std::cout << "未知传感器数据类型: " << static_cast<int>(sensorData.type) << std::endl;
                break;
            }
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


/**
 * 测试接口，集成时可根据情况删除
 * @brief 键盘输入处理线程函数
 * @param flight 飞行控制器（共享指针，确保线程安全访问）
 */
void handleKeyboardInput(std::shared_ptr<DroneSDK::FlightController> flight) {
    char input;
    std::cout << "\n===== 操作提示 =====" << std::endl;
    std::cout << "输入 1 : 执行手动模式指令" << std::endl;
    std::cout << "输入 q : 退出程序" << std::endl;
    std::cout << "====================\n" << std::endl;

    while (!g_exitFlag) {
        std::cin >> input;  // 阻塞等待键盘输入（独立线程，不影响SDK主线程）
        
        // 输入1：执行起飞指令
        if (input == '1') {
            flight->enableVirtualStickMode(true);
            std::cout << "输入 1 : 执行手动模式(开关虚拟摇杆模式)指令" << std::endl;
        }
        else if(input == '2'){
            flight->arm();
            std::cout << "输入 2 : 执行解锁指令" << std::endl;
        }
        else if(input == '3'){
            flight->takeOff(90);  // 执行起飞
            std::cout << "输入 3 : 执行起飞指令（目标高度200米）" << std::endl;
        }
        else if(input == '4'){
            flight->land();  // 执行降落
            std::cout << "输入 4 : 执行降落指令" << std::endl;
        }
        else if(input == '5'){
            flight->setFlightMode(FlightModeControl::AUTO_MISSION);  // 执行自动巡航
            std::cout << "输入 5 : 执行自动巡航指令" << std::endl;
        }
        else if(input == '6'){
            double latitude = 22.4124518; 
            double longitude = 113.5628897;
            float altitude = 190.0;
            flight->guideToPosition(latitude, longitude, altitude);
            std::cout << "输入 6 : 执行指点飞行至某位置指令" << std::endl;
        }
        // 输入q/Q：退出程序
        else if (input == 'q' || input == 'Q') {
            std::cout << "\n[提示] 收到退出指令，程序即将结束..." << std::endl;
            g_exitFlag = true;  // 设置退出标志，主线程循环终止
        }
        // 无效输入提示
        else {
            std::cout << "\n[提示] 无效输入！请输入1执行起飞，输入q退出程序。" << std::endl;
        }
    }
}


int main() {
    DroneSDK::ZiyanSdk sdk;
    DroneSDK::SDKConfig config;
    config.logLevel = "info";
    config.enableLogging = true;
    config.logPath = "./logs";
    config.autoConnect = true;
    config.eventLoopHandler = std::make_shared<SDKListener>();

    if (!sdk.initializeWithConfig(config)) {
        std::cerr << "SDK init failed: " << sdk.getLastError() << std::endl;
        return -1;
    }

    auto flight = sdk.getFlightController();
    if (flight) {
        std::cout << "Flight controller ready" << std::endl;
        //flight->takeOff(200);
    }

    // 测试接口，集成时可根据情况删除 --- 创建键盘输入处理线程（分离主线程，避免阻塞SDK事件循环）
    std::thread inputThread(handleKeyboardInput, flight);

    //std::this_thread::sleep_for(std::chrono::seconds(1));

    while (!g_exitFlag)//true
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    inputThread.join();

    sdk.shutdown();
    std::cout << "SDK shutdown" << std::endl;
    return 0;
}

