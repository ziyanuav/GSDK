#include <nlohmann/json.hpp>
#include "GsdkManager/ziyanSdk.h"
#include "GsdkManager/sdkInterface.h"
#include <iostream>
#include <thread>
#include <atomic>
#include <fstream>
#include <cstdint>
#include <iomanip>

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
                static std::ofstream filewrite("swarm_dump_29_rtp.264", std::ios::binary);
                
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
                std::cout << "姿态数据: 俯仰角=" << std::fixed << std::setprecision(1)
                      << sensorData.data.attitude.pitch << "° 滚转角=" << sensorData.data.attitude.roll
                      << "° 偏航角=" << sensorData.data.attitude.yaw << "°" << std::endl;
                break;
            }

        case SENSOR_VELOCITY:
            {
                std::cout << "速度数据: X轴=" << std::fixed << std::setprecision(1)
                      << sensorData.data.velocity.vx << "m/s Y轴=" << sensorData.data.velocity.vy
                      << "m/s Z轴=" << sensorData.data.velocity.vz << "m/s" << std::endl;
                break;
            }

        case SENSOR_BATTERY:
            {
                std::cout << "电池数据: 电量=" << sensorData.data.battery.percentage << "% 电压="
                      << sensorData.data.battery.voltage << "V 电流=" << sensorData.data.battery.current << "A" << std::endl;
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
                std::cout << "当前控制端是否有控制权(false:无控制权，true：有控制权): " << lf.control_authority_has_control << std::endl;

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
            {
                std::cout << "未知传感器数据类型: " << static_cast<int>(sensorData.type) << std::endl;
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
void handleKeyboardInput(std::shared_ptr<DroneSDK::FlightController> flightController, 
                         std::shared_ptr<DroneSDK::PayloadController> payload,
                         std::shared_ptr<DroneSDK::VideoTransmission> videoHandle,
                         std::shared_ptr<DroneSDK::FlightParameterManager> flightParameter,
                        std::shared_ptr<DroneSDK::SensorDataManager> sensorManager) {
    char input;
    std::cout << "\n===== 操作提示 =====" << std::endl;
    std::cout << "输入 1 (回车) : 执行手动模式指令" << std::endl;
    std::cout << "输入 q : 退出程序" << std::endl;
    std::cout << "====================\n" << std::endl;

    while (!g_exitFlag) {
        std::cin >> input;  // 阻塞等待键盘输入（独立线程，不影响SDK主线程）
        
        if (input == '0') {
            flightController->enableVirtualStickMode(false);
            std::cout << "输入 0 : 执行急停模式(开关虚拟摇杆模式)指令" << std::endl;
        }
        else if (input == '1') {
            flightController->enableVirtualStickMode(true);
            std::cout << "输入 1 : 执行手动模式(开关虚拟摇杆模式)指令" << std::endl;
        }
        else if(input == '2'){
            flightController->arm();
            std::cout << "输入 2 : 执行解锁指令" << std::endl;
        }
        else if(input == '3'){
            flightController->takeOff(90);  // 执行起飞
            std::cout << "输入 3 : 执行起飞指令（目标高度90米）" << std::endl;
        }
        else if(input == '4'){
            flightController->land();  // 执行降落
            std::cout << "输入 4 : 执行降落指令" << std::endl;
        }
        else if(input == '5'){
            flightController->setFlightMode(FlightModeControl::AUTO_MISSION);  // 执行自动巡航
            std::cout << "输入 5 : 执行自动巡航指令" << std::endl;
        }
        else if(input == '6'){
            double latitude = 22.4123646; 
            double longitude = 113.5622541;
            float altitude = 90.0;
            flightController->guideToPosition(latitude, longitude, altitude);
            std::cout << "输入 6 : 执行指点飞行至某位置指令" << std::endl;
        }
        else if(input == '7'){
            flightController->returnToHome();
            std::cout << "输入 7 : 执行直线返航指令" << std::endl;
        }
        else if(input == '8'){
            flightController->takeControlAuthority();
            std::cout << "输入 8 : 执行当前遥控器端获取控制权切换指令(夺取控制权是否成功，查看属性参数：control_authority_has_control)" << std::endl;
        }
        else if(input == '9'){
            flightController->setVirtualStick(0,0,1,0);
            std::cout << "输入 9 : 向前移动" << std::endl;
        }
        else if(input == 'a'){
            flightController->setVirtualStick(0,0,0,0);
            std::cout << "输入 a : 向前停止" << std::endl;
        }
        else if(input == 'b'){
            payload->modeGimbal(3);  // 执行云台模式切换
            std::cout << "输入 b : 执行云台模式切换指令" << std::endl;
        }
        else if(input == 'c'){
            flightController->startMission();
            std::cout << "输入 c : 开始执行航线任务" << std::endl;
        }
        else if(input == 'd'){
            flightController->pauseMission();
            std::cout << "输入 d : 暂停执行航线任务" << std::endl;
        }
        else if(input == 'e'){
            flightController->resumeMission();
            std::cout << "输入 e : 恢复执行航线任务" << std::endl;
        }
        else if(input == 'f'){
            flightController->stopMission();
            std::cout << "输入 f : 终止执行航线任务" << std::endl;
        }
        else if(input == 'g'){
            //1. 切换到FPV摄像头
            std::cout << "\n--- 切换到FPV摄像头 ---" << std::endl;
            DroneSDK::VideoSourceInfo fpvSource;
            fpvSource.type = DroneSDK::VideoSourceType::FPV;
            fpvSource.index = 0;
            
            bool switchResult = videoHandle->switchVideoSource(fpvSource);
            std::cout << "切换到FPV摄像头: " << (switchResult ? "成功" : "失败") << std::endl;
        }
        else if(input == 'h'){
            // 2. 切换到云台摄像头（索引1）
            std::cout << "\n--- 切换到云台摄像头（索引1）---" << std::endl;
            DroneSDK::VideoSourceInfo gimbalSource1;
            gimbalSource1.type = DroneSDK::VideoSourceType::GIMBAL;
            gimbalSource1.index = 1;
            
            bool switchResult = videoHandle->switchVideoSource(gimbalSource1);
            std::cout << "切换到云台摄像头1: " << (switchResult ? "成功" : "失败") << std::endl;
        }
        else if(input == 'i'){
  
            std::cout << "\n--- 读取参数列表 ---" << std::endl;

            std::vector<std::string> paramNames = {"RTL_ALT", "BATT_LOW_VOLT", "FENCE_ALT_MAX"};

            flightParameter->readBatchParamList(paramNames, [](const DroneSDK::BatchParameterReadResult& result)
            {
                // ===== 上层回调：拿到返回结果，打印数据 =====
                std::cout << "[上层调用readBatchParamList] 批量参数返回" << std::endl;
                for (const auto& item : result.statusList)
                {
                    std::cout << "[上层] paramId:" << item.paramId
                                << ", value:" << item.value
                                << ", success:" << item.success
                                << ", msg:" << item.errorMessage
                                << std::endl;
                }
            });

        }
        else if(input == 'j'){
            // 4. 设置返航高度
            std::cout << "\n--- 设置返航高度为95m ---" << std::endl;

            flightParameter->writeRtlAlt(95,
                                            [](const DroneSDK::ParameterStatus &status)
                                            {
                                                std::cout << "  参数ID: " << status.paramId << std::endl;
                                                std::cout << "  设置值: " << status.value << " cm" << std::endl;
                                                std::cout << "  设置成功: " << (status.success ? "是" : "否") << std::endl;
                                                if (!status.success)
                                                {
                                                    std::cout << "  错误信息: " << status.errorMessage << std::endl;
                                                }
                                            });

        }
        else if(input == 'k'){
            // 5. Gcs通信丢失保护使能开关
            std::cout << "\n--- Gcs通信丢失保护使能开关 ---" << std::endl;
            flightParameter->writeFsGcsEnable(true, 2,
                                            [](const DroneSDK::ParameterStatus &status)
                                            {
                                                std::cout << "  参数ID: " << status.paramId << std::endl;
                                                std::cout << "  设置值: " << status.value << std::endl;
                                                std::cout << "  设置成功: " << (status.success ? "是" : "否") << std::endl;
                                                if (!status.success)
                                                {
                                                    std::cout << "  错误信息: " << status.errorMessage << std::endl;
                                                }
                                            });

        }
        else if(input == 'l'){
            std::cout << "\n--- 载荷控制接口 ---" << std::endl;

            // 1. 设置云台姿态（俯仰、左右）-- 持续控制直到接收到停止指令才停止移动
            std::cout << "设置云台姿态移动 (pitch=10°, yaw=20°): ";
            bool gimbalResult = payload->setGimbalAttitude(10.0f, 20.0f);
            // 持续杆量,若不需要持续控制,请将此句代码调用
            //payload->setGimbalAttitude(0, 0);
            std::cout << (gimbalResult ? "成功" : "失败") << std::endl;
        }
        else if(input == 'm'){
            std::cout << "设置云台姿态移动停止 (pitch=0°, yaw=0°): ";
            bool gimbalResult = payload->setGimbalAttitude(0.0f, 0.0f);
            std::cout << (gimbalResult ? "成功" : "失败") << std::endl;
        }
        else if(input == 'n'){
            // 云台校准
            std::cout << "云台自动校准: ";
            bool calibrateResult = payload->calibrateGimbal();
            std::cout << (calibrateResult ? "成功" : "失败") << std::endl;

            // // 云台重置
            // std::cout << "云台重置参数: ";
            // bool resetResult = payload->resetGimbal();
            // std::cout << (resetResult ? "成功" : "失败") << std::endl;

            // // 云台回中模式切换
            // std::cout << "云台回中模式: ";
            // bool modeResult = payload->modeGimbal(3);
            // std::cout << (modeResult ? "成功" : "失败") << std::endl;

            // // 云台正摄模式
            // std::cout << "云台正摄模式: ";
            // modeResult = payload->modeGimbal(11);
            // std::cout << (modeResult ? "成功" : "失败") << std::endl;

        }
        else if(input == 'o'){
            // 相机控制调试
            std::cout << "\n--- 相机控制调试 ---" << std::endl;

            // 7. 拍照
            std::cout << "拍照: ";
            bool photoResult = payload->takePhoto();
            std::cout << (photoResult ? "成功" : "失败") << std::endl;

            // 8. 开始录像
            // std::cout << "开始录像: ";
            // bool startRecordResult = payload->startRecording();
            // std::cout << (startRecordResult ? "成功" : "失败") << std::endl;

            // // 9. 停止录像
            // std::cout << "停止录像: ";
            // bool stopRecordResult = payload->stopRecording();
            // std::cout << (stopRecordResult ? "成功" : "失败") << std::endl;

        }
        else if(input == 'p'){
            // 变焦功能调试
            std::cout << "\n--- 变焦功能调试 ---" << std::endl;

            // 10. 数字变焦（即：相机绝对变焦）
            std::cout << "数字变焦（即：相机绝对变焦） (zoom=2.0x): ";
            bool digitalZoomResult = payload->digitalZoom(2.0f);
            std::cout << (digitalZoomResult ? "成功" : "失败") << std::endl;


            // 12. 光学速度变焦（即：持续变焦）--- （备注 -> 1:放大 0:停止 -1:缩小）
            std::cout << "光学速度变焦 （即：持续变焦）(speed=1.0): ";
            bool opticalSpeedZoomResult = payload->opticalSpeedZoom(1);
            std::cout << (opticalSpeedZoomResult ? "成功" : "失败") << std::endl;
        }
        else if(input == 'r'){
            // 自定义控件调试
            std::cout << "\n--- 自定义控件调试 ---" << std::endl;

            // 14. 获取自定义控件数据
            std::cout << "获取所有自定义控件json数据: ";
            payload->getCustomWidgetData([](const json &data)
                                                { std::cout << "收到控件数据: " << data.dump() << std::endl; });
            std::cout << "请求已发送" << std::endl;

            // 13. 设置自定义控件值
            // std::cout << "设置自定义控件值 (index=1, widget_index=\"29\", type=4, value=2): ";//"value": //0：广角   1：变焦   2：红外   3：拼接
            // bool setWidgetResult = payload->setCustomWidgetValue(1, "29", 4, 2);
            // std::cout << (setWidgetResult ? "成功" : "失败") << std::endl;
            
        }
        else if(input == 's'){

            std::cout << "激光测距";
            bool isOpen = true;
            bool result = payload->setCamMeasure(isOpen);
            std::cout << (result ? "成功" : "失败") << std::endl;

            std::cout << "框选目标跟踪";
            int track_cmd = 1; //1:开始跟踪  2:结束跟踪
            int track_mode = 1; //  1:框选跟踪  2:目标名称跟踪
            float startX = 0.50f; //框选跟踪时矩形框左上角 x 坐标 （归一化 0.0–1.0）
            float startY =  0.50f; //框选跟踪时矩形框左上角 y 坐标 （归一化 0.0–1.0）
            float endX =  0.50f;  //框选跟踪时矩形框右下角 x 坐标 （归一化 0.0–1.0）
            float endY =  0.50f; //框选跟踪时矩形框右下角 y 坐标 （归一化 0.0–1.0）
            bool trackResult = payload->setCamTrack(track_cmd, track_mode, startX, startY, endX, endY);
            std::cout << (trackResult ? "成功" : "失败") << std::endl;

        }
        else if(input == 't'){

            std::cout << "播放TTS语音";
            std::string ttsText = "紧急通知，请立即疏散";
            bool result1 = payload->setTtsPlay(ttsText);
            std::cout << (result1 ? "成功" : "失败") << std::endl;

            std::cout << "停止Opus或TTS音频播放";
            bool result2 = payload->opusOrTtsAudioStop();
            std::cout << (result2 ? "成功" : "失败") << std::endl;

            std::cout << "设置云台角度";
            float pitch = -15;
            bool result3 = payload->setOpusAngle(pitch);
            std::cout << (result3 ? "成功" : "失败") << std::endl;

            std::cout << "设置音量";
            int volume = 50;
            bool result4 = payload->setOpusVolume(volume);
            std::cout << (result4 ? "成功" : "失败") << std::endl;

            std::cout << "设置循环模式（0：单次  1：循环）";
            int mode = 1;
            bool result5 = payload->setOpusLoopMode(mode);
            std::cout << (result5 ? "成功" : "失败") << std::endl;

        }
        else if(input == 'u'){

            std::cout << "获取卫星数";
            std::cout << "sensorManager getSatelliteCount=" << sensorManager->getSatelliteCount() << std::endl;

        }

        else if(input == 'v'){
            //指定参数名称读取/写入测试例子
            // 读取返航高度参数
            // std::cout << "\n--- 读取返航高度参数 ---" << std::endl;
            // flightParameter->readParameter(DroneSDK::FlightParameterType::RTL_ALT,
            //                             [](const DroneSDK::ParameterStatus &status)
            //                             {
            //                                 std::cout << "  参数ID: " << status.paramId << std::endl;
            //                                 std::cout << "  参数值: " << status.value << std::endl;
            //                                 std::cout << "  读取成功: " << (status.success ? "是" : "否") << std::endl;
            //                                 if (!status.success)
            //                                 {
            //                                     std::cout << "  错误信息: " << status.errorMessage << std::endl;
            //                                 }
            //                             });

            // // 4. 设置返航高度
            // std::cout << "\n--- 设置返航高度为1000cm ---" << std::endl;
            // flightParameter->writeParameter(DroneSDK::FlightParameterType::RTL_ALT, 1000.0f,
            //                                 [](const DroneSDK::ParameterStatus &status)
            //                                 {
            //                                     std::cout << "  参数ID: " << status.paramId << std::endl;
            //                                     std::cout << "  设置值: " << status.value << " cm" << std::endl;
            //                                     std::cout << "  设置成功: " << (status.success ? "是" : "否") << std::endl;
            //                                     if (!status.success)
            //                                     {
            //                                         std::cout << "  错误信息: " << status.errorMessage << std::endl;
            //                                     }
            //                                 });      
        }

        else if(input == 'z'){
            // 构造 Mission 示例并上传，带进度回调
            {

                Mission mission;
                mission.routeType = "waypoint";// `waypoint` 普通航点航线，`polygon` 建图航线，`corridor` 带状航线，`circleCruise` 圆周巡航
                mission.referAlt = "RA_ALTITUDE";// `RA_ALTITUDE` 相对高度；`AL_ALTITUDE` 海拔高度；`RA_TERRAIN` 地形高度 |
                mission.takeoffHeight = 55.0f;//起飞高度
                mission.finishedAction = "STRAIGHT_RETURN";//STRAIGHT_RETURN` 直线返航；`ORIGINAL_RETURN` 原路返航；`LAND` 降落；
                mission.descentSpeed = 1.0f;// 航线下降速度
                mission.ascentSpeed = 2.0f; // 航线上升速度
                mission.horiSpeed = 9.0f;   // 航线水平速度

                // 航点1：普通航点
                MissionPoint w1;
                w1.position.latitude = 22.4115423;
                w1.position.longitude = 113.5625717;
                w1.position.altitude = 50.0;  //航点高度设置
                w1.actionType = DroneSDK::PointCommand::WAYPOINT;
                w1.params["param1"] = 0.0f;
                w1.params["param2"] = 0.0f;
                w1.params["param3"] = 0.0f;
                w1.params["param4"] = 0.0f;
                mission.waypoints.push_back(w1);

                // 航点命令2：悬停航点，param1 = 悬停秒数  备注：“飞到该航点坐标，然后悬停”
                MissionPoint w2;
                w2.position.latitude = 22.4128811;
                w2.position.longitude = 113.5624920;
                w2.position.altitude = 60.0; //航点高度设置
                w2.actionType = DroneSDK::PointCommand::LOITER;
                w2.params["param1"] = 5.0f;  // 悬停5秒
                w2.params["param2"] = 0.0f;
                w2.params["param3"] = 0.0f;
                w2.params["param4"] = 0.0f;
                mission.waypoints.push_back(w2);

                // 航点3：普通航点
                MissionPoint w3;
                w3.position.latitude = 22.4126641;
                w3.position.longitude = 113.5613759;
                w3.position.altitude = 65.0; //航点高度设置
                w3.actionType = DroneSDK::PointCommand::WAYPOINT;
                w3.params["param1"] = 0.0f;
                w3.params["param2"] = 0.0f;
                w3.params["param3"] = 0.0f;
                w3.params["param4"] = 0.0f;
                mission.waypoints.push_back(w3);
                
                // 航点命令4：拍照命令（在飞到上一个航点坐标时执行“拍照命令”）
                MissionPoint w4;
                w4.position.latitude = 0.0;
                w4.position.longitude = 0.0;
                w4.position.altitude = 0.0;
                w4.actionType = DroneSDK::PointCommand::DIGICAM_SHOT;
                w4.params["param1"] = 0.0f;
                w4.params["param2"] = 0.0f;
                w4.params["param3"] = 0.0f;
                w4.params["param4"] = 0.0f;
                mission.waypoints.push_back(w4);

                // 航点5：云台姿态控制命令（在飞到上一个航点坐标时执行“云台姿态控制命令”）
                MissionPoint w5;
                w5.position.latitude = 0.0;
                w5.position.longitude = 0.0;
                w5.position.altitude = 0.0;
                w5.actionType = DroneSDK::PointCommand::MOUNT_CONTROL; // 云台控制 p1 俯仰角度（-90至90），p3 偏移角（-90至90）
                w5.params["param1"] = 20.0f;                           // 俯仰角度（范围值：-90至90）
                w5.params["param2"] = 0.0f;                            
                w5.params["param3"] = 30.0f;                           // 偏移角（范围值：-90至90）
                w5.params["param4"] = 0.0f;                            
                mission.waypoints.push_back(w5);

                // 测绘航点6：测绘第一个航点---开启   (多个测绘航点一起使用，不能混合用其它航点或者命令)
                // MissionPoint w6;
                // w6.position.latitude = 39.9072;
                // w6.position.longitude = 116.3104;
                // w6.position.altitude = 70.0; //航点高度设置
                // w6.actionType = DroneSDK::PointCommand::SURVEY;        // 测绘航点 p1 是否开启拍照（1：开，0：跟随上一个，2：关）,p2 等距离(单位米)/等时间（单位s），p4 （0：距离类型1：时间类形）
                // w6.params["param1"] = 1.0f;                           // 开启拍照
                // w6.params["param2"] = 20.0f;                            // 等距离/等时间间隔
                // w6.params["param3"] = 0.0f;                           
                // w6.params["param4"] = 0.0f;                            // 0 距离模式，1 时间模式
                // mission.waypoints.push_back(w6);

                // 测绘航点7：举例-测绘中间多个航点---跟随状态   (多个测绘航点一起使用，不能混合用其它航点或者命令)
                // MissionPoint w7;
                // w7.position.latitude = 39.9072;
                // w7.position.longitude = 116.4104;
                // w7.position.altitude = 70.0; //航点高度设置
                // w7.actionType = DroneSDK::PointCommand::SURVEY;        // 测绘航点 p1 是否开启拍照（1：开，0：跟随上一个，2：关）,p2 等距离(单位米)/等时间（单位s），p4 （0：距离类型1：时间类形）
                // w7.params["param1"] = 0.0f;                              // 跟随
                // w7.params["param2"] = 20.0f;                            // 等距离/等时间间隔
                // w7.params["param3"] = 0.0f;                           
                // w7.params["param4"] = 0.0f;                            // 0 距离模式，1 时间模式
                // mission.waypoints.push_back(w7);

                // 测绘航点8：测绘最后一个航点---停止 (多个测绘航点一起使用，不能混合用其它航点或者命令)
                // MissionPoint w8;
                // w8.position.latitude = 39.9072;
                // w8.position.longitude = 116.5104;
                // w8.position.altitude = 70.0; //航点高度设置
                // w8.actionType = DroneSDK::PointCommand::SURVEY;        // 测绘航点 p1 是否开启拍照（1：开，0：跟随上一个，2：关）,p2 等距离(单位米)/等时间（单位s），p4 （0：距离类型1：时间类形）
                // w8.params["param1"] = 2.0f;                           // 停止拍照
                // w8.params["param2"] = 20.0f;                            // 等距离/等时间间隔
                // w8.params["param3"] = 20.0f;                           
                // w8.params["param4"] = 0.0f;                            // 0 距离模式，1 时间模式
                // mission.waypoints.push_back(w8);

                std::cout << "Uploading mission with " << mission.waypoints.size() << " waypoints..." << std::endl;
                bool ok = flightController->uploadMission(
                    mission,
                    [](int current, int total)
                    {
                        std::cout << "Mission upload progress: " << current << "/" << total << std::endl;
                    });
                std::cout << (ok ? "Mission upload request sent." : "Mission upload failed to send.") << std::endl;

                if (ok)
                {
                    //bool started = flightController->startMission();
                    //std::cout << (started ? "Mission started." : "Mission start failed.") << std::endl;
                }
            }
            std::cout << "输入 z : 上传航线任务至飞行器" << std::endl;
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

    auto SDKManagerImpl = sdk.getSDKManager();
    auto flight = sdk.getFlightController();
    auto payload = sdk.getPayloadController();
    auto videoHandle = sdk.getVideoTransmission();
    auto flightParameter = sdk.getFlightParameterController();
    auto sensorManager = sdk.getSensorDataManager();

    if (flight) {
        std::cout << "Flight controller ready" << std::endl;
        //flight->takeOff(200);
    }
    //备注：目前该版本已经具备AB点控制权切换功能，该GSDK端默认作为B点，默认无控制权（control_authority_has_control），如果需要控制权需要调用接口夺取控制权：flight->takeControlAuthority()
    //如果需要默认就有控制权，可沟通紫燕技术人员处理

    // 测试接口，集成时可根据情况删除 --- 创建键盘输入处理线程（分离主线程，避免阻塞SDK事件循环）
    //std::thread inputThread(handleKeyboardInput, flight, payload, videoHandle, flightParameter, sensorManager);

    //std::this_thread::sleep_for(std::chrono::seconds(1));

    while (!g_exitFlag)//true
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 测试接口线程启动
    //inputThread.join();

    sdk.shutdown();
    std::cout << "SDK shutdown" << std::endl;
    return 0;
}

