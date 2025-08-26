#include "mqtt_protocol.h"
#include "board.h"
#include "application.h"
#include "settings.h"
#include "system_info.h"

#include <esp_log.h>
#include <cstring>
#include <arpa/inet.h>
#include <nvs_flash.h>
#include <algorithm>
#include <memory>
#include "assets/lang_config.h"

#define TAG "MQTT"

MqttProtocol::MqttProtocol() {
    event_group_handle_ = xEventGroupCreate();
}

MqttProtocol::~MqttProtocol() {
    ESP_LOGI(TAG, "MqttProtocol deinit");
    vEventGroupDelete(event_group_handle_);
}

bool MqttProtocol::Start() {
    return StartMqttClient(false);
}

bool MqttProtocol::StartMqttClient(bool report_error) {
    if (mqtt_ != nullptr) {
        ESP_LOGW(TAG, "Mqtt client already started");
        mqtt_.reset();
    }

    Settings settings("mqtt", false);
    auto endpoint = settings.GetString("endpoint");
    auto client_id = settings.GetString("client_id");
    auto username = settings.GetString("username");
    auto password = settings.GetString("password");
    int keepalive_interval = settings.GetInt("keepalive", 240);
    publish_topic_ = settings.GetString("publish_topic");
    subscribe_topic_ = settings.GetString("subscribe_topic");

    // Legacy user id based on MAC (decimal) for composing topics
    user_id3_ = SystemInfo::GetMacAddressDecimal();
    phone_control_topic_ = std::string("doll/control/") + user_id3_;
    languagesType_topic_ = std::string("doll/set/") + user_id3_;
    moan_topic_ = std::string("doll/control_moan/") + user_id3_;

    ESP_LOGI(TAG, "Device ID (MAC decimal): %s", user_id3_.c_str());

    // If publish_topic_ is empty, build it like old version: stt/doll/{user_id3}/{language}
    if (publish_topic_.empty()) {
        auto lang = LoadLanguageTypeFromNVS("zh");
        publish_topic_ = std::string("stt/doll/") + user_id3_ + "/" + lang;
        ESP_LOGI(TAG, "Publish topic (legacy): %s", publish_topic_.c_str());
    }

    if (endpoint.empty()) {
        ESP_LOGW(TAG, "MQTT endpoint is not specified");
        if (report_error) {
            SetError(Lang::Strings::SERVER_NOT_FOUND);
        }
        return false;
    }

    // 保存当前语言，便于后续 UpdateLanguage 使用（仅当走旧规则时）
    if (publish_topic_.rfind("stt/doll/", 0) == 0) {
        // 按旧规则生成的，解析出语言部分（最后一个'/'后的内容）
        auto pos = publish_topic_.find_last_of('/');
        if (pos != std::string::npos && pos + 1 < publish_topic_.size()) {
            languagesType_ = publish_topic_.substr(pos + 1);
        }
    }

    auto network = Board::GetInstance().GetNetwork();
    mqtt_ = network->CreateMqtt(0);
    mqtt_->SetKeepAlive(keepalive_interval);

    mqtt_->OnDisconnected([this]() {
        ESP_LOGI(TAG, "Disconnected from endpoint");
    });

    mqtt_->OnMessage([this, phone_control_topic = phone_control_topic_, languagesType_topic = languagesType_topic_, moan_topic = moan_topic_](const std::string& topic, const std::string& payload) {
        // 仅在收到 JSON 时输出日志；其它消息降为 DEBUG，避免播放/音频相关噪声日志
        if (!payload.empty() && payload[0] == '{') {
            ESP_LOGI(TAG, "JSON: %s", payload.c_str());
        } else {
            ESP_LOGD(TAG, "Non-JSON on %s (%u bytes)", topic.c_str(), (unsigned)payload.size());
        }

        // 1) 可选：忽略 VAD 专用主题（本版不处理 VAD）
        // 若后续需要，可在此添加 VAD 主题处理逻辑

        // 2) 处理原有主题（与旧版一致）
        if (topic == subscribe_topic_) {
            // 如果是JSON消息 (以'{'开头)
            if (!payload.empty() && payload[0] == '{') {
                ESP_LOGD(TAG, "Received JSON message: %s", payload.c_str());
                if (on_incoming_json_ != nullptr) {
                    cJSON* root = cJSON_Parse(payload.c_str());
                    if (root != nullptr) {
                        on_incoming_json_(root);
                        cJSON_Delete(root);
                    }
                }
            } else {
                // 否则，视为音频数据包（服务器发送纯OPUS payload），封装为 AudioStreamPacket
                if (on_incoming_audio_ != nullptr) {
                    auto packet = std::make_unique<AudioStreamPacket>();
                    packet->sample_rate = 16000;
                    packet->frame_duration = OPUS_FRAME_DURATION_MS;
                    packet->timestamp = 0; // 纯MQTT下行，服务端可选用外部时序；此处填0
                    packet->payload.resize(payload.size());
                    memcpy(packet->payload.data(), payload.data(), payload.size());
                    on_incoming_audio_(std::move(packet));
                }
            }
        }
        else if (topic == phone_control_topic) {
            // 处理手机控制消息
            ESP_LOGD(TAG, "Received control message: %s", payload.c_str());
            cJSON* root = cJSON_Parse(payload.c_str());
            if (root != nullptr) {
                if (on_incoming_json_ != nullptr) {
                    on_incoming_json_(root);
                }
                cJSON_Delete(root);
            }
        }
        else if (topic == languagesType_topic) {
            // 处理语言设置消息
            ESP_LOGD(TAG, "Received language setting: %s", payload.c_str());
            cJSON* root = cJSON_Parse(payload.c_str());
            if (root != nullptr) {
                if (on_incoming_json_ != nullptr) {
                    on_incoming_json_(root);
                }
                cJSON_Delete(root);
            }
        } else if (topic == moan_topic) {
            // 处理呻吟声控制消息
            ESP_LOGI(TAG, "Received moan: %s", payload.c_str());
            cJSON* root = cJSON_Parse(payload.c_str());
            if (root != nullptr) {
                if (on_incoming_json_ != nullptr) {
                    on_incoming_json_(root);
                }
                cJSON_Delete(root);
            }
        } else {
          ESP_LOGD(TAG, "Unhandled topic: %s", topic.c_str());
        }
    });

    ESP_LOGI(TAG, "Connecting to MQTT broker: %s", endpoint.c_str());
    std::string broker_address;
    int broker_port = 1883; // 默认与旧版一致，使用非TLS端口
    size_t pos = endpoint.find(':');
    if (pos != std::string::npos) {
        broker_address = endpoint.substr(0, pos);
        broker_port = std::stoi(endpoint.substr(pos + 1));
    } else {
        broker_address = endpoint;
    }
    if (!mqtt_->Connect(broker_address, broker_port, client_id, username, password)) {
        ESP_LOGE(TAG, "Failed to connect to endpoint");
        SetError(Lang::Strings::SERVER_NOT_CONNECTED);
        return false;
    }

    ESP_LOGI(TAG, "Connected to endpoint");

    // 统一打印 MQTT 主题配置
    ESP_LOGI(TAG, "MQTT Topics:");
    ESP_LOGI(TAG, "  Publish: %s", publish_topic_.c_str());
    if (!subscribe_topic_.empty()) {
        ESP_LOGI(TAG, "  Subscribe: %s", subscribe_topic_.c_str());
    }
    ESP_LOGI(TAG, "  Control: %s", phone_control_topic_.c_str());
    ESP_LOGI(TAG, "  Language: %s", languagesType_topic_.c_str());
    ESP_LOGI(TAG, "  Moan: %s", moan_topic_.c_str());

    // 旧版对齐：分主题订阅
    if (!subscribe_topic_.empty()) {
        mqtt_->Subscribe(subscribe_topic_, 1);
        ESP_LOGI(TAG, "Subscribed to %s (QoS 1)", subscribe_topic_.c_str());
    }
    if (!phone_control_topic_.empty()) {
        mqtt_->Subscribe(phone_control_topic_, 0);
        ESP_LOGI(TAG, "Subscribed to %s (QoS 0)", phone_control_topic_.c_str());
    }
    if (!languagesType_topic_.empty()) {
        mqtt_->Subscribe(languagesType_topic_, 0);
        ESP_LOGI(TAG, "Subscribed to %s (QoS 0)", languagesType_topic_.c_str());
    }
    if (!moan_topic_.empty()) {
        mqtt_->Subscribe(moan_topic_, 0);
        ESP_LOGI(TAG, "Subscribed to %s (QoS 0)", moan_topic_.c_str());
    }

    return true;
}

// 对齐旧项目：当收到语言变更时，动态更新发布主题（仅在未配置自定义 publish_topic 时生效）
void MqttProtocol::UpdateLanguage(const std::string& language) {
    languagesType_ = language;
    if (publish_topic_.empty() || publish_topic_.rfind("stt/doll/", 0) == 0) {
        // 主题按旧规则：stt/doll/{user_id3}/{language}
        if (user_id3_.empty()) {
            user_id3_ = SystemInfo::GetMacAddressDecimal();
        }
        publish_topic_ = std::string("stt/doll/") + user_id3_ + "/" + language;
        ESP_LOGI(TAG, "Updated publish topic to: %s, language: %s", publish_topic_.c_str(), language.c_str());
    } else {
        ESP_LOGI(TAG, "Custom publish_topic is set, skip updating: %s", publish_topic_.c_str());
    }
}

bool MqttProtocol::SendText(const std::string& text) {
    if (publish_topic_.empty()) {
        return false;
    }
    if (!mqtt_->Publish(publish_topic_, text)) {
        ESP_LOGE(TAG, "Failed to publish message: %s", text.c_str());
        SetError(Lang::Strings::SERVER_ERROR);
        return false;
    }
    return true;
}

bool MqttProtocol::SendAudio(std::unique_ptr<AudioStreamPacket> packet) {
    // 纯 MQTT 模式：分片发送（参考旧项目）
    if (publish_topic_.empty() || mqtt_ == nullptr || !mqtt_->IsConnected()) {
        ESP_LOGE(TAG, "MQTT client not connected or topic empty");
        return false;
    }

    const size_t MAX_CHUNK_SIZE = 1024; // 每片最大1KB
    const char* data_ptr = reinterpret_cast<const char*>(packet->payload.data());
    size_t total = packet->payload.size();

    if (total <= MAX_CHUNK_SIZE) {
        std::string chunk(data_ptr, total);
        if (!mqtt_->Publish(publish_topic_, chunk, 0)) {
            ESP_LOGE(TAG, "Failed to publish audio chunk (single)");
            return false;
        }
        return true;
    }

    size_t remaining = total;
    size_t offset = 0;
    size_t total_chunks = (total + MAX_CHUNK_SIZE - 1) / MAX_CHUNK_SIZE;
    ESP_LOGI(TAG, "Sending audio in chunks: total=%u, chunks=%u", (unsigned)total, (unsigned)total_chunks);

    while (remaining > 0) {
        size_t chunk_size = std::min(remaining, MAX_CHUNK_SIZE);
        std::string chunk(data_ptr + offset, chunk_size);
        if (!mqtt_->Publish(publish_topic_, chunk, 0)) { // QoS0 低延迟
            ESP_LOGE(TAG, "Failed to publish audio chunk at offset %u", (unsigned)offset);
            return false;
        }
        remaining -= chunk_size;
        offset += chunk_size;
    }

    return true;
}

void MqttProtocol::CloseAudioChannel() {
    // 纯 MQTT 模式下，仅发送 goodbye 控制消息
    std::string message = "{";
    message += "\"session_id\":\"" + session_id_ + "\",";
    message += "\"type\":\"goodbye\"";
    message += "}";
    SendText(message);

    if (on_audio_channel_closed_ != nullptr) {
        on_audio_channel_closed_();
    }
}

bool MqttProtocol::OpenAudioChannel() {
    // 纯 MQTT 模式：无需等待 UDP 握手，认为通道随 MQTT 连接即可用
    if (mqtt_ == nullptr || !mqtt_->IsConnected()) {
        ESP_LOGI(TAG, "MQTT is not connected, try to connect now");
        if (!StartMqttClient(true)) {
            return false;
        }
    }

    error_occurred_ = false;
    // 可选：发送一个上行控制消息通知开始（按需实现）

    if (on_audio_channel_opened_ != nullptr) {
        on_audio_channel_opened_();
    }
    return true;
}

std::string MqttProtocol::GetHelloMessage() {
    // 纯 MQTT 模式：可选的握手消息（按需由服务器决定是否使用）
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "hello");
    cJSON_AddStringToObject(root, "transport", "mqtt");
    auto json_str = cJSON_PrintUnformatted(root);
    std::string message(json_str);
    cJSON_free(json_str);
    cJSON_Delete(root);
    return message;
}


std::string MqttProtocol::LoadLanguageTypeFromNVS(const std::string& default_lang) {
    std::string saved_language;
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("config", NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        size_t required_size = 0;
        err = nvs_get_str(nvs_handle, "languagesType", NULL, &required_size);
        if (err == ESP_OK && required_size > 1) {
            std::string tmp;
            tmp.resize(required_size);
            if (nvs_get_str(nvs_handle, "languagesType", tmp.data(), &required_size) == ESP_OK) {
                if (!tmp.empty() && tmp.back() == '\0') tmp.pop_back();
                saved_language = tmp;
            }
        }
        nvs_close(nvs_handle);
    }
    if (saved_language.empty()) saved_language = default_lang;
    return saved_language;
}


bool MqttProtocol::IsAudioChannelOpened() const {
    return mqtt_ != nullptr && mqtt_->IsConnected() && !error_occurred_ && !IsTimeout();
}
