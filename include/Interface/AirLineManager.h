#ifndef AIR_LINE_H
#define AIR_LINE_H
#include <stdlib.h>
#include <deque>
#include <string>
#include <functional>

#include "Interface/vehicle.h"

struct __mavlink_message;

enum CommandType
{
    COMMAND_WAYPOINT = 16,         // 普通航点
    COMMAND_SURVEY = 13,           // 测绘航点 p1 是否开启拍照1开0关,p2 等距离/等时间，p4 0距离类型1时间类形
    COMMAND_WAYPOINT_JUMP = 177,   // 航点跳转 p1跳转航点,p2跳转次数
    COMMAND_SPEED_CHANGE = 178,    // 飞行速度变更 p2速度 米每秒
    COMMAND_MOUNT_CONTROL = 205,   // 云台控制 p1 俯仰角度，p3 偏移角
    COMMAND_DIGICAM_SHOT = 203,    // 拍照
    COMMAND_TAKEOFF = 22,          // 起飞
    COMMAND_RETURN_TO_LAUNCH = 20, // 直线返航
    COMMAND_LAND = 21,             // 原地降落
};
struct Waypoint
{
    double latitude;
    double longitude;
    double alt;
    uint16_t seq;        // 序号
    CommandType command; // 航点类型
    float param1;        // 相关参数
    float param2;
    float param3;
    float param4;
};
using ProgressCallBack = std::function<void(int current, int total)>;

enum OptionType
{
    UPLOAD,
    DOWNLOAD
};

class AirLineManager
{
public:
    int uploadMission(std::string &jsonOfWaypoints, ProgressCallBack callback);
    void downloadMission(ProgressCallBack callback);

    friend class Vehicle;
    
#ifdef INCLUDE_JINGANSDK
    friend class JaWaylinesFlightTaskMission;
#endif


private:
    AirLineManager(Vehicle *drone);
    void handleMessage(const __mavlink_message *msg);
    void uploadMission(std::deque<Waypoint> &waypoints, ProgressCallBack callback);

    Vehicle *DroneDevice;
    std::deque<Waypoint> upload_waypoints;
    std::vector<Waypoint> waypoints_;
    ProgressCallBack downloadCallback_;
    ProgressCallBack uploadCallBack_;
    OptionType m_optiontype;
    int m_mission_total;
    int m_mission_seq;
    void missionRequest(int count);
    void uploadMissionItem(Waypoint wp);
};


#endif