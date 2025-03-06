#ifndef JOY_STICK_H
#define JOY_STICK_H


#include <stdlib.h>
#include <atomic>
#include <functional>


#include "Interface/vehicle.h"

struct JoyStickProperites
{
    std::atomic<int16_t> x; // 飞机前后
    std::atomic<int16_t> y; // 飞机左右
    std::atomic<int16_t> z; // 飞机爬升 下降
    std::atomic<int16_t> r; // 飞机角度偏移
    JoyStickProperites() : x(0), y(0), z(0), r(0) {}
};
struct PodAttiProperites{
    std::atomic<int16_t> pitch;//俯仰
    std::atomic<int16_t> yaw;//偏移
    PodAttiProperites() : pitch(0), yaw(0){}
};

class JoyStickManager
{
public:
    virtual ~JoyStickManager();
    void postJoyProperites(int16_t &x, int16_t &y, int16_t &z, int16_t &r);
    void postPodAtiProperites(int16_t &pitch,int16_t &yaw);

    friend class Vehicle;
    

private:
    JoyStickManager(Vehicle *DroneDev);

    int64_t mJoyCancelDelay=90;
    int64_t mJoyPostDelay = 90;
    void HandlerJoystickMsg();//发送飞机摇杆包
    void HandlerPodAttiMsg();//发送吊舱
    void CancelJoyStickPro();//取消摇杆赋值
    void CancelPodAttiPro();//取消吊舱姿态控制赋值
    std::thread joyStikcThread; //飞机姿态控制线程
    std::thread cancelJoyStickThread;//取消无人机控制粘键
    std::thread canclePodAttiThread;//取消吊舱控制粘键
    std::thread podAttiThread; //吊舱调仓控制线程
    Vehicle *DroneDevice;
    bool mRun;
    uint8_t *joyBuffer; // 发送摇杆的buffer
    uint8_t *podBuffer; // 发送吊舱的buffer
    uint16_t lenJoy;
    uint16_t lenPod;
};


#endif