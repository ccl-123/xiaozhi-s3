#ifndef MQTT_PROTOCOL_H
#define MQTT_PROTOCOL_H


#include "protocol.h"
#include <mqtt.h>
#include <cJSON.h>
#include "qmi8658.h"
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include "system_info.h"

#include <functional>
#include <string>
#include <map>
#include <mutex>

#define MQTT_PING_INTERVAL_SECONDS 90
#define MQTT_RECONNECT_INTERVAL_MS 10000

#define MQTT_PROTOCOL_SERVER_HELLO_EVENT (1 << 0)

class MqttProtocol : public Protocol {
public:
    MqttProtocol();
    ~MqttProtocol();

    bool Start() override;
    bool SendAudio(std::unique_ptr<AudioStreamPacket> packet) override;
    bool OpenAudioChannel() override;
    void CloseAudioChannel() override;
    bool IsAudioChannelOpened() const override;

    // 对齐旧项目：运行时更新语言时，动态更新发布主题
    void UpdateLanguage(const std::string& language);

    // IMU数据发送方法
    void SendImuStatesAndValue(const t_sQMI8658& imu_data, int touch_value = 0);

private:
    EventGroupHandle_t event_group_handle_;

    // MQTT topics
    std::string publish_topic_;
    std::string subscribe_topic_;
    std::string phone_control_topic_;
    std::string languagesType_topic_;
    std::string moan_topic_;

    // 保存当前语言，便于日志与持久化
    std::string languagesType_;

    // Device ID used in topic composition (MAC in decimal, legacy style)
    std::string user_id3_;

    std::mutex channel_mutex_;
    std::unique_ptr<Mqtt> mqtt_;

    bool StartMqttClient(bool report_error=false);

    // Legacy helper: load language type from NVS (namespace "config", key "languagesType")
    std::string LoadLanguageTypeFromNVS(const std::string& default_lang = "zh");

    bool SendText(const std::string& text) override;
    std::string GetHelloMessage();
};


#endif // MQTT_PROTOCOL_H
