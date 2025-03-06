#ifndef PARAM_H
#define PARAM_H

#include <map>
#include <functional>

#include "Interface/vehicle.h"

// 参数操作回调类型
using ParamCallback = std::function<void(const std::string &, float, bool)>;
using ParamListCallback = std::function<void(const std::map<std::string, float>&)>;

struct __mavlink_param_value_t;

class ParamOptHandler
{
public:
    ParamOptHandler(const std::string &param_id,
                    float param_value,
                    const std::string &opt_type,
                    ParamCallback callback,
                    std::function<void()> send_func);
    virtual ~ParamOptHandler();
    // 校验参数值并确认成功
    bool checkAndConfirm(float received_value);

private:
    void sendRequest();
    bool isValueEqual(float received) const;
    void setupTimers();
    void scheduleTimer(int delay_ms, std::function<void()> action);
    void executeCallback(float param_val, bool success);
    void stop();

    std::string param_id;
    float param_value_;
    std::string opt_type;
    ParamCallback callback_;
    std::function<void()> send_func_;
    std::mutex mtx_;
    std::atomic<bool> active_;
};

class ParamManager
{
public:
    enum ParameterType
    {
        RTL_ALT, ///< 返航高度。取值范围：30~300000，单位厘米，增量1
        // AVOID_ENABLE, ///< 避障
        FS_GCS_ENABLE, ///< 失联动作。取值：1 - 降落；2 - 悬停
        WPNAV_SPEED, ///< 飞行速度。取值范围：50~2000，单位厘米，增量50
    };

    virtual ~ParamManager();

    int setParameter(ParameterType paramType, float paramValue, ParamCallback callback);

    int RequestSingleParam(ParameterType paramType, ParamCallback callback);

    void RequestParamList(ParamListCallback callback);

    friend class Vehicle;

    
private:
    ParamManager(Vehicle *DroneDev);

    void RequestSingleParam(const std::string &param_id, ParamCallback callback);
    void setParameter(const std::string &param_id, float param_value, ParamCallback callback);
    void handleParamValue(const __mavlink_param_value_t& param);

    void removeHandler(const std::string &param_id, const std::string &map_type);
    void sendParamRequest();
    void completeCollection();
    void timeoutHandler();
    void cleanup();

    Vehicle *DroneDevice;
    std::map<std::string, float> param_list;
    ParamListCallback requestParamListCallback;
    std::atomic<bool> is_request_param_list;
    std::atomic<uint16_t> expected_count_;
    std::atomic<uint16_t> received_count_;
    std::thread timeout_thread_;
    std::unordered_map<std::string, std::shared_ptr<ParamOptHandler>> settingParamMap;
    std::unordered_map<std::string, std::shared_ptr<ParamOptHandler>> readingParamMap;
    std::mutex map_mutex_;
};
#endif