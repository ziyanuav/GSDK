#ifndef _VEHICLE_PUBLIC_H_
#define _VEHICLE_PUBLIC_H_

#define GCS_SYSID 255
#define GCS_COMPID 200
#define TARGET_SYSID 1
#define TARGET_COMPID 1
#define DEFAULT_TAKEOFF_ALT_M 50

// Auto Pilot Modes enumeration
enum AutoPilotMode
{
    STABILIZE = 0,           // manual airframe angle with manual throttle
    ACRO = 1,                // manual body-frame angular rate with manual throttle
    ALT_HOLD = 2,            // manual airframe angle with automatic throttle
    AUTO = 3,                // fully automatic waypoint control using mission commands
    GUIDED = 4,              // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
    LOITER = 5,              // automatic horizontal acceleration with automatic throttle
    RTL = 6,                 // automatic return to launching point
    CIRCLE = 7,              // automatic circular flight with automatic throttle
    LAND = 9,                // automatic landing with horizontal position control
    DRIFT = 11,              // semi-autonomous position, yaw and throttle control
    SPORT = 13,              // manual earth-frame angular rate control with manual throttle
    FLIP = 14,               // automatically flip the vehicle on the roll axis
    AUTOTUNE = 15,           // automatically tune the vehicle's roll and pitch gains
    POSHOLD = 16,            // automatic position hold with manual override, with automatic throttle
    BRAKE = 17,              // full-brake using inertial/GPS system, no pilot input（紫燕变更模式功能为安全模式，此模式主要保证在切入该模式时，使飞行器处于当前状态下的最安全情况）
    THROW = 18,              // throw to launch mode using inertial/GPS system, no pilot input
    AVOID_ADSB = 19,         // automatic avoidance of obstacles in the macro scale - e.g. full-sized aircraft
    GUIDED_NOGPS = 20,       // guided mode but only accepts attitude and altitude
    SMART_RTL = 21,          // SMART_RTL returns to home by retracing its steps
    FLOWHOLD = 22,           // FLOWHOLD holds position with optical flow without rangefinder
    FOLLOW = 23,             // follow attempts to follow another vehicle or ground station
    ZIGZAG = 24,             // ZIGZAG mode is able to fly in a zigzag manner with predefined point A and point B
    SYSTEMID = 25,           // System ID mode produces automated system identification signals in the controllers
    AUTOROTATE = 26,         // Autonomous autorotation
    AUTO_RTL = 27,           // Auto RTL, this is not a true mode, AUTO will report as this mode if entered to perform a DO_LAND_START Landing sequence
    TURTLE = 28,             // Flip over after crash
    LINKAGE = 61,            // 联动模式，紫燕自定义模式
    MANUAL = 60,             // 手控模式，紫燕自定义模式
    SWARM_MANUAL = 80,       // 集群手控
    SWARM_GUIDED = 81,       // 集群引导
    SWARM_AUTO = 82,         // 集群自动航线
    SWARM_ATTACK = 83,       // 集群自动攻击
    AVIONICS_OVERHAUL = 127, // 检修模式，紫燕自定义模式
};

enum class DroneOperationStatus
{
    WAIT_FOR_START,                      // idle
    SEND_SETMODE_GUIDED,                 // 发送设置模式为GUIDED
    SEND_ARM_CMD,                        // 发送ARM指令
    WAIT_FOR_ARM_COMPLETE,               // 等待无人机解锁
    SEND_TAKEOFF_CMD,                    // 发送起飞指令
    WAIT_FOR_TAKEOFF_COMPLETE,           // 等待无人机起飞完成
    SEND_SETMODE_RTL,                    // 发送设置模式为RTL
    SEND_SETMODE_LAND,                   // 发送设置模式为LAND
    TRIGGER_LOGIC_TAKEOFF,               // 触发起飞逻辑
    WAIT_FOR_LOGIC_TAKEOFF_COMPLETE,     // 等待起飞逻辑完成
    SEND_SETMODE_AUTO,                   // 发送设置模式为AUTO
    WAIT_FOR_SETMODE_COMPLETE,           // 等待无人机设置模式完成
    SEND_MISSION_ITEM,                   // 发送MISSION_ITEM报文，传递目标点
    WAIT_FOR_SEND_MISSION_ITEM_COMPLETE, // 等待发送MISSION_ITEM报文完成
    LOGIC_COMPLETE,                      // 逻辑完成
    SEND_SETMODE,
    WAIT_FOR_RPM                         // 等待转速达到起飞条件
};

enum class MissionType
{
    Mission_All,
    Auto_Mission,
    Guided_Mission,
    RTL_Mission,
    Takeoff_Mission,
    Land_Mission,
    Mode_Change_Action
};


#endif