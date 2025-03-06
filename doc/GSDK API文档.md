# GSDK API文档

## 一、Vehicle

| 说明  | 飞机指令                           |
| --- | ------------------------------ |
| 头文件 | #include <Interface/vehicle.h> |

**环境准备：**

```
sudo apt-get install libssl-dev
sudo apt install libcurl4-openssl-dev
sudo apt-get install libzip-dev
sudo apt-get install libtinyxml2-dev
sudo apt-get install  ffmpeg
```

**集成实例：**

```
#include <iostream>
#include "Interface/vehicle.h"

int main()
{
	// std::string ipAddress = "192.168.1.40";
	// int port = 9003;
	std::string ipAddress = "192.168.44.128";
	int port = 9003;
	std::cout<<ipAddress<<" : "<<port<<std::endl;
	Vehicle *device = new Vehicle(ipAddress, port);
	device->Run();

	getchar();
	return 0;
}



/*
控制台输出以下内容，表示连机成功
Running in simulation mode.
Vehicle armed sate:0system_status:3,mode:0
Mode changed from 0to 9
BroadcastSignal
BroadcastSignal
Vehicle armed sate:0system_status:5,mode:9
Vehicle::HandleRun,SignalSent=0
BroadcastSignal
Vehicle armed sate:Vehicle::HandleRun0system_status:3,mode:,SignalSent=90
Vehicle armed sate:0system_status:3,mode:9
Vehicle armed sate:0system_status:3,mode:9
Vehicle armed sate:0system_status:3,mode:9
Vehicle armed sate:0system_status:3,mode:9
Vehicle armed sate:0system_status:3,mode:9
Vehicle armed sate:0system_status:3,mode:9
Vehicle armed sate:0system_status:3,mode:9
Vehicle armed sate:0system_status:3,mode:9
......
*/
```

**Public Functions:**

- Vehicle(std::string ipAddress, int &port);
  
  - 说明：构造函数
  
  - 参数：
    
    - ipAddress ip地址
    
    - port           端口号

- void Run();
  
  - 说明：启动

- void Stop();
  
  - 说明：停止

- VehicleInfo getInfo();
  
  - 说明：获取设备信息
  
  - 返回值：设备信息结构体
    
    - VehicleInfo 说明：
      
      - bool armed;              // 是否已解锁
      
      - bool landed_state;       // 是否正在飞行
      
      - uint32_t mode;           // 无人机飞行模式
      
      - float battery_voltage;   // 电池电压
      
      - float battery_current;   // 电池电流
      
      - float battery_remaining; // 电池剩余百分比
      
      - float latitude;          // 纬度
      
      - float longitude;         // 经度
      
      - float altitude;          // 高度
      
      - float roll;              // 滚转角
      
      - float pitch;             // 俯仰角
      
      - float yaw;               // 偏航角
      
      - float main_rpm;          // 主桨转速
      
      - int satellites_visible;  // 可见卫星数

**Public Attributes:**

- JoyStickManager *JoyStickClient;
  
  - 说明：摇杆控制

- PodManager *podManager;
  
  - 说明：吊舱能力

- AirLineManager *airLineManager;
  
  - 说明：航线管理

- ParamManager *paramManager;
  
  - 说明：飞控参数管理

---

## 二、JoyStickManager

| 说明  | 摇杆控制                                   |
| --- | -------------------------------------- |
| 头文件 | #include <Interface/JoyStickManager.h> |
| 备注  | 只能通过Vehicle访问，不能直接实例化对象                |

**Public Functions：**

- void postJoyProperites(int16_t &x, int16_t &y, int16_t &z, int16_t &r);
  
  - 说明：控制飞机姿态
  
  - 参数：
    
    - x X轴，归一化为范围[-1000,1000]。INT16_MAX的值表示此轴无效。通常对应于操纵杆上的向前（1000）、向后（-1000）移动，和vehicle的俯仰
    
    - y Y轴，归一化为范围[-1000,1000]。INT16_MAX的值表示此轴无效。通常对应于操纵杆上的左（-1000）、右（1000）移动，和vehicle的侧倾
    
    - z Z轴，归一化为范围[-10001000]。INT16_MAX的值表示此轴无效。通常对应于一个单独的滑块运动，操纵杆上的最大值为1000，最小值为-1000，以及vehicle的推力。正值是正推力，负值是负推力。
    
    - r R轴，归一化为范围[-10001000]。INT16_MAX的值表示此轴无效。通常对应于操纵杆的扭转，逆时针为1000，顺时针为-1000，以及vehicle的偏航。

- void postPodAtiProperites(int16_t &pitch,int16_t &yaw);
  
  - 说明：控制吊舱姿态
  
  - 参数：
    
    - pitch  俯仰
    
    - yaw    偏航

---

## 三、PodManager

| 说明  | 吊舱能力                              |
| --- | --------------------------------- |
| 头文件 | #include <Interface/PodManager.h> |
| 备注  | 只能通过Vehicle访问，不能直接实例化对象           |

**Public Functions:**

- void changePodZoom(int focal);
  
  - 说明：更改摄像头焦距
  
  - 参数：
    
    - focal

- void changePodStyle(PodActionType imageType);
  
  - 说明：更改摄像头成像
  
  - 参数：
    
    - imageType

---

## 四、AirLineManager

| 说明  | 航线管理                                  |
| --- | ------------------------------------- |
| 头文件 | #include <Interface/AirLineManager.h> |
| 备注  | 只能通过Vehicle访问，不能直接实例化对象               |

**Public Functions:**

- int uploadMission(std::string &jsonOfWaypoints, ProgressCallBack callback);
  
  - 说明：上传航线
  
  - 参数：
    
    - jsonOfWaypoints 包含航线信息的json字符串，详细说明见下文
    
    - callback

- 返回值：成功返回0，失败返回错误码

- void downloadMission(ProgressCallBack callback);
  
  - 说明：下载航线
  
  - 参数：
    
    - callback

---

## 五、ParamManager

| 说明  | 飞控参数管理                              |
| --- | ----------------------------------- |
| 头文件 | #include <Interface/ParamManager.h> |
| 备注  | 只能通过Vehicle访问，不能直接实例化对象             |

**Public Functions:**

- int setParameter(ParameterType paramType, float paramValue, ParamCallback callback);
  
  - 说明：执行参数设置功能
  
  - 参数：
    
    - paramType            参数类型
    
    - paramValue           参数值
    
    - ParamCallback      根据此回调判断参数是否成功设置
  
  - 返回值：0表示已执行参数设置功能，-1表示参数值不合法，-2表示参数类型异常

- void RequestParamList(ParamListCallback callback);
  
  - 说明：读取所有参数值
  
  - 参数：
    
    - callback 在回调中获取参数值

-  void RequestSingleParam(ParameterType paramType, ParamCallback callback);
  
  - 说明：
  
  - 参数：
    
    - paramType    参数类型
    
    - callback          在回调中获取参数值
  
  - 返回值：返回值：0表示已执行参数设置功能，-2表示参数类型异常

---

## 六、枚举说明

1、**MissionType**

- Mission_All

- Auto_Mission

- Guided_Mission

- RTL_Mission

- Takeoff_Mission

- Land_Mission

- Mode_Change_Action

2、**PodActionType**

- ABS_IN_OUT        输入输出 都是绝对倍速

- INFRARED            红外成像

- VISIBLE_LIGHT    可见光成像

3、**ParamManager::ParameterType**

- RTL_ALT        返航高度。取值范围：30~300000，单位厘米，增量1

- FS_GCS_ENABLE 失联动作。取值：1 - 降落；2 - 悬停

- WPNAV_SPEED    飞行速度。取值范围：50~2000，单位厘米，增量50
  
  ---

## 七、航线json说明：

| 参数             | 说明          | 取值范围                                                                                                                                                                                                                                                | 备注                                                          |
| -------------- | ----------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------- |
| itemId         | 唯一id        |                                                                                                                                                                                                                                                     |                                                             |
| name           | 航线名称        |                                                                                                                                                                                                                                                     |                                                             |
| takeOffHeight  | 起飞高度        | 0～4000米                                                                                                                                                                                                                                             |                                                             |
| descentSpeed   | 下降高度        | 0.1～10 m/s                                                                                                                                                                                                                                          |                                                             |
| ascentSpeed    | 爬升速度        | 0.1～10 m/s                                                                                                                                                                                                                                          |                                                             |
| horiSpeed      | 水平速度        | 0.1～17 m/s                                                                                                                                                                                                                                          |                                                             |
| flightAltitude | 航线高度        | 0～4000米                                                                                                                                                                                                                                             |                                                             |
| afterFinish    | 航线执行完毕后执行动作 | STRAIGHT_RETURN:  直线返航，  ORIGINAL_RETURN：原路返航，  LAND:降落                                                                                                                                                                                             |                                                             |
| rtlAlt         | 返航高度        | 0～4000米                                                                                                                                                                                                                                             |                                                             |
| referAlt       | 高度参考        | RA_ALTITUDE:相对高度， AL_ALTITUDE:海拔高度                                                                                                                                                                                                                  |                                                             |
| createName     | 创建人         |                                                                                                                                                                                                                                                     |                                                             |
| editTime       | 创建时间        |                                                                                                                                                                                                                                                     |                                                             |
| wayPoints      | 航点集合        |                                                                                                                                                                                                                                                     |                                                             |
| index          | 航点脚标/指令脚标   |                                                                                                                                                                                                                                                     | 若type 非航点类型而是指令类型 则代表指令脚标 ，即该指令飞到该脚标位的航点后执行，若有多个指令即按其集合顺序执行 |
| actCmd         | 指令参数        | [0,0,0,0] 分别代表param1,param2,param3,param4                                                                                                                                                                                                           |                                                             |
| height         | 航点高度        |                                                                                                                                                                                                                                                     |                                                             |
| latitude       | 纬度          |                                                                                                                                                                                                                                                     |                                                             |
| longitude      | 经度          |                                                                                                                                                                                                                                                     |                                                             |
| type           | 航点类型/指令类型   | 航点类型：<br>WAYPOINT-普通直线航点<br>SURVEY-测绘航点  <br><br>指令类型：<br>WAYPOINT_JUMP-航点跳转 <br>- - - -param1跳转航点 param2跳转次数 <br><br>SPEED_CHANGE-航速改变<br>- - - -param2速度单位（m/s）<br><br>MOUNT_CONTROL-云台控制<br>- - - -param1 俯仰角度，param3 偏移角<br><br>DIGICAM_SHOT-拍照 |                                                             |

```json
{

        "itemId": "312fd684-66a0-48cc-be70-10eff1fd1589", 
        "name": "测试航线",
        "takeOffHeight": 30,
        "descentSpeed": 4,
        "ascentSpeed": 3,
        "horiSpeed": 3,
        "flightAltitude": 100,
        "afterFinish": "ROUTE_RETURN",
        "rtlAlt": 15,
        "referAlt": "RA_ALTITUDE",  
        "createName": "林怡",
        "editTime": "2024-11-27 13:56:21", 
        "wayPoints": [
          {
            "index": 0,
            "actCmd": [0,0,0,0],
            "height": 100,
            "latitude": 22.3822229,
            "longitude": 113.5364597,
            "type": "WPT_LINE"
          },
            {
            "index": 0,
            "actCmd": [0,0,0,0],
            "height": 0,
            "latitude":0,
            "longitude": 0,
            "type": "DIGICAM_SHOT"
          }

        ]
      }
```
