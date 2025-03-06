#ifndef _FW_VEHICLE_DEF__
#define _FW_VEHICLE_DEF__

#include "framework/common/uthread.h"

#include <unordered_map>
#include <thread>
#include <inttypes.h>
#include <iostream>
#include <string>
#include <condition_variable>
#include <map>

#include "Vehicle/MissionProcessor.h"
#include "Vehicle_public.h"


#define GCS_SYSID 255
#define GCS_COMPID 200
#define TARGET_SYSID 1
#define TARGET_COMPID 1
#define DEFAULT_TAKEOFF_ALT_M 50

struct __mavlink_message;


class JoyStickManager;
class PodManager;
class AirLineManager;
class ParamManager;

class SocketConnect;
class RtspClientObj;

struct VehicleInfo
{
    VehicleInfo():
        armed(false),
        landed_state(true),
        mode(0),
        battery_voltage(0.0f),
        battery_current(0.0f),
        battery_remaining(0.0f),
        latitude(0.0f),
        longitude(0.0f),
        altitude(0.0f),
        roll(0.0f),
        pitch(0.0f),
        yaw(0.0f)
    {}
    
    bool armed;              // 是否已解锁
    bool landed_state;       // 是否正在飞行
    uint32_t mode;           // 无人机飞行模式
    float battery_voltage;   // 电池电压
    float battery_current;   // 电池电流
    float battery_remaining; // 电池剩余百分比
    float latitude;          // 纬度
    float longitude;         // 经度
    float altitude;          // 高度
    float roll;              // 滚转角
    float pitch;             // 俯仰角
    float yaw;               // 偏航角
    float main_rpm;          // 主桨转速
    int satellites_visible;  // 可见卫星数
};

class Vehicle : public UThread
{
public:
    Vehicle(std::string ipAddress, int &port);
    virtual ~Vehicle();
    void Run();
    void Stop();

    VehicleInfo getInfo();

    JoyStickManager *JoyStickClient;
    PodManager *podManager;
    AirLineManager *airLineManager;
    ParamManager *paramManager;

    friend class JoyStickManager;
    friend class PodManager;
    friend class AirLineManager;
    friend class ParamManager;

    friend class AutoMissionPlan;
    friend class GuidedMissionPlan;
    // friend class LandMissionPlan;
    // friend class RtlMissionPlan;
    friend class TakeOffMissionPlan;
    friend class ModeChangeAction;

#ifdef INCLUDE_JINGANSDK
    friend class JaTakeoffMission;
    friend class JaStopAllMission;
    friend class JaAutolandMission;
    friend class JaGohomeMission;
    friend class JaGotoPositionMission;
    friend class JaLiveLensChangeMission;
    friend class JaLiveZoomChangeMission;
    friend class JaMoveGimbalMission;
    friend class JaMoveUavMission;
    friend class JaPlatform;
    friend class JaWaylinesFlightTaskMission;
    friend class JaDroneProperties;
#endif

    friend class CloudGohomeMission;
    friend class CloudAutolandMission;
    friend class CloudStopAllMission;
    friend class CloudGotoPositionMission;
    friend class CloudLiveZoomChangeMission;
    friend class CloudMoveGimbalMission;
    friend class CloudTakeoffMission;
    friend class DroneStatus;
    friend class CloudStopAllMission;
    friend class CloudStopAllMission;


private:
    void ParseLinkMsg(const __mavlink_message &msg);
    void BroadcastSignal(MissionType missionID);
    void WaitMsgToProcess();
    virtual void HandleRun(void *param);

    DroneOperationStatus CurrentStatus;
    AutoPilotMode CurrentMode;
    SocketConnect *Linkclient;
    RtspClientObj *VideoSrcClient;

    // std::vector<std::unique_ptr<MissionProcessor>> missionProcessors;
    std::unordered_map<MissionType, std::unique_ptr<MissionProcessor>> missionMap;
    VehicleInfo info;
    struct
    {
        std::chrono::system_clock::time_point last_mission_accepted;
        std::chrono::system_clock::time_point last_fence_accepted;
        std::chrono::system_clock::time_point last_param_accepted;
        std::chrono::system_clock::time_point last_all_accepted;
    } last_ack_accepted; // 回复状态

    std::mutex mtx;
    std::condition_variable cv;
    bool SignalSent = false;
    pthread_t mPid;
    bool mRun;
    uint8_t *buffer;
    uint16_t len;
    bool buf_ready;
    volatile int missionId;
    std::chrono::system_clock::time_point last_heartbeat; // 最后一次心跳时间
};

#endif
