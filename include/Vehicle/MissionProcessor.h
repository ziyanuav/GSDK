#ifndef _MISSIONPROCESSOR_H_
#define _MISSIONPROCESSOR_H_


#include <bits/stdint-uintn.h>
#include <chrono>
#include <string>
#include <vector>

#include "Interface/Vehicle_public.h"


class MissionProcessor
{
public:
    virtual void Processcmd(uint8_t *buf, uint16_t &len, bool &buf_ready) = 0;
    virtual DroneOperationStatus GetRuningStatus() = 0;
    virtual void start(std::vector<std::string> &command_params) = 0;
    DroneOperationStatus MissionStatus;
    std::chrono::_V2::steady_clock::time_point StartTime;
    int GcsSysID;
    int GcsComID;
    int TargetSysID;
    int TargetComPID;
    std::string MissionName;
};

#endif