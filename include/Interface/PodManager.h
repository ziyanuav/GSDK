#ifndef POD_H
#define POD_H


#include <stdlib.h>
#include "Interface/vehicle.h"


enum PodFunType
{
    ZOOM = 3, // 焦距功能
    IMAGE = 8 // 成像模式
};


enum PodActionType
{
    ABS_IN_OUT = 3,   // 输入输出 都是绝对倍速
    INFRARED = 1,     // 红外成像
    VISIBLE_LIGHT = 3 // 可见光成像
};

class PodManager
{
public:
    void changePodZoom(int focal);                // 更改摄像头焦距
    void changePodStyle(PodActionType imageType); // 更改摄像头成像

    friend class Vehicle;
    
private:
    PodManager(Vehicle *DroneDev);
    Vehicle *DroneDevice;
};


#endif