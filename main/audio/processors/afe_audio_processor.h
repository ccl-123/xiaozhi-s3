#ifndef AFE_AUDIO_PROCESSOR_H
#define AFE_AUDIO_PROCESSOR_H

#include <esp_afe_sr_models.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#include <string>
#include <vector>
#include <functional>

#include "audio_processor.h"
#include "audio_codec.h"

class AfeAudioProcessor : public AudioProcessor {
public:
    AfeAudioProcessor();
    ~AfeAudioProcessor();

    void Initialize(AudioCodec* codec, int frame_duration_ms) override;
    void Feed(std::vector<int16_t>&& data) override;
    void Start() override;
    void Stop() override;
    bool IsRunning() override;
    void OnOutput(std::function<void(std::vector<int16_t>&& data)> callback) override;
    void OnVadStateChange(std::function<void(bool speaking)> callback) override;
    size_t GetFeedSize() override;
    void EnableDeviceAec(bool enable) override;

    // VAD能量阈值动态调整
    void SetVadEnergyThreshold(float threshold_dbfs);
    void SetVadSmoothFactor(float smooth_factor);
    void SetVadMinEnergyFrames(int min_frames);
    float GetCurrentEnergyLevel() const;
    float GetVadEnergyThreshold() const;

private:
    EventGroupHandle_t event_group_ = nullptr;
    esp_afe_sr_iface_t* afe_iface_ = nullptr;
    esp_afe_sr_data_t* afe_data_ = nullptr;
    std::function<void(std::vector<int16_t>&& data)> output_callback_;
    std::function<void(bool speaking)> vad_state_change_callback_;
    AudioCodec* codec_ = nullptr;
    int frame_samples_ = 0;
    bool is_speaking_ = false;
    std::vector<int16_t> output_buffer_;

    // VAD能量检测相关
    float current_energy_dbfs_ = -100.0f;     // 当前音频能量（dBFS）
    float smoothed_energy_dbfs_ = -100.0f;    // 平滑后的能量值
    int energy_above_threshold_frames_ = 0;   // 连续超过阈值的帧数
    int energy_below_threshold_frames_ = 0;   // 连续低于阈值的帧数
    float vad_energy_threshold_dbfs_ = -40.0f; // 🎯 核心：能量阈值
    float vad_energy_smooth_factor_ = 0.5f;   // 🎯 能量平滑因子
    int vad_min_energy_frames_ = 2;           // 🎯 连续帧数阈值
    bool vad_cache_just_processed_ = false;   // 🎯 标记是否刚处理了VAD缓存

    void AudioProcessorTask();

    // VAD能量检测辅助函数
    float CalculateRMSEnergy(const int16_t* data, size_t samples);
    float ConvertToDBFS(float rms);
    bool CheckEnergyThreshold(float energy_dbfs);
};

#endif 