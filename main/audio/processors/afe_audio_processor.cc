#include "afe_audio_processor.h"
#include <esp_log.h>
#include <cmath>

#define PROCESSOR_RUNNING 0x01

#define TAG "AfeAudioProcessor"

AfeAudioProcessor::AfeAudioProcessor()
    : afe_data_(nullptr) {
    event_group_ = xEventGroupCreate();

    // 预分配输出缓冲区，避免频繁重新分配
    output_buffer_.reserve(8192);  // 预分配8KB空间

    // 打印内存信息
    ESP_LOGI(TAG, "AFE constructor - Free heap: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "AFE constructor - Free internal heap: %" PRIu32 " bytes", esp_get_free_internal_heap_size());
}

void AfeAudioProcessor::Initialize(AudioCodec* codec, int frame_duration_ms) {
    codec_ = codec;
    frame_samples_ = frame_duration_ms * 16000 / 1000;

    // Pre-allocate output buffer capacity
    output_buffer_.reserve(frame_samples_);

    int ref_num = codec_->input_reference() ? 1 : 0;

    std::string input_format;
    for (int i = 0; i < codec_->input_channels() - ref_num; i++) {
        input_format.push_back('M');
    }
    for (int i = 0; i < ref_num; i++) {
        input_format.push_back('R');
    }

    srmodel_list_t *models = esp_srmodel_init("model");
    char* ns_model_name = esp_srmodel_filter(models, ESP_NSNET_PREFIX, NULL);
    char* vad_model_name = esp_srmodel_filter(models, ESP_VADN_PREFIX, NULL);
    
    afe_config_t* afe_config = afe_config_init(input_format.c_str(), NULL, AFE_TYPE_VC, AFE_MODE_HIGH_PERF);
    afe_config->aec_mode = AEC_MODE_VOIP_HIGH_PERF;
    afe_config->vad_mode = VAD_MODE_1;  // 数值越大触发概率越高
    afe_config->vad_min_noise_ms = 800;  // 800ms静音时长，降低误触发（官方推荐1000ms）
    
    // 添加更多VAD调优参数以降低灵敏度（使用ESP-SR实际支持的参数）
    afe_config->vad_min_speech_ms = 128;  // 语音段的最短持续时间（毫秒）
    afe_config->vad_delay_ms = 128;       // VAD首帧触发到语音首帧数据的延迟量
    
    if (vad_model_name != nullptr) {
        afe_config->vad_model_name = vad_model_name;
    }

    if (ns_model_name != nullptr) {
        afe_config->ns_init = true;
        afe_config->ns_model_name = ns_model_name;
        afe_config->afe_ns_mode = AFE_NS_MODE_NET;
    } else {
        afe_config->ns_init = false;
    }

    afe_config->afe_perferred_core = 1;
    afe_config->afe_perferred_priority = 1;
    afe_config->agc_init = false;
    afe_config->memory_alloc_mode = AFE_MEMORY_ALLOC_MORE_PSRAM;

#ifdef CONFIG_USE_DEVICE_AEC
    afe_config->aec_init = true;
    afe_config->vad_init = true;  // 强制启用VAD以支持Speaking状态下的打断检测
#else
    afe_config->aec_init = false;
    afe_config->vad_init = true;
#endif

    afe_iface_ = esp_afe_handle_from_config(afe_config);
    afe_data_ = afe_iface_->create_from_config(afe_config);
    
    xTaskCreate([](void* arg) {
        auto this_ = (AfeAudioProcessor*)arg;
        this_->AudioProcessorTask();
        vTaskDelete(NULL);
    }, "audio_communication", 4096, this, 3, NULL);
}

AfeAudioProcessor::~AfeAudioProcessor() {
    if (afe_data_ != nullptr) {
        afe_iface_->destroy(afe_data_);
    }
    vEventGroupDelete(event_group_);
}

size_t AfeAudioProcessor::GetFeedSize() {
    if (afe_data_ == nullptr) {
        return 0;
    }
    return afe_iface_->get_feed_chunksize(afe_data_);
}

void AfeAudioProcessor::Feed(std::vector<int16_t>&& data) {
    if (afe_data_ == nullptr) {
        return;
    }
    afe_iface_->feed(afe_data_, data.data());
}

void AfeAudioProcessor::Start() {
    xEventGroupSetBits(event_group_, PROCESSOR_RUNNING);
}

void AfeAudioProcessor::Stop() {
    xEventGroupClearBits(event_group_, PROCESSOR_RUNNING);
    if (afe_data_ != nullptr) {
        afe_iface_->reset_buffer(afe_data_);
    }
}

bool AfeAudioProcessor::IsRunning() {
    return xEventGroupGetBits(event_group_) & PROCESSOR_RUNNING;
}

void AfeAudioProcessor::OnOutput(std::function<void(std::vector<int16_t>&& data)> callback) {
    output_callback_ = callback;
}

void AfeAudioProcessor::OnVadStateChange(std::function<void(bool speaking)> callback) {
    vad_state_change_callback_ = callback;
}

void AfeAudioProcessor::AudioProcessorTask() {
    auto fetch_size = afe_iface_->get_fetch_chunksize(afe_data_);
    auto feed_size = afe_iface_->get_feed_chunksize(afe_data_);
    ESP_LOGI(TAG, "Audio communication task started, feed size: %d fetch size: %d",
        feed_size, fetch_size);

    while (true) {
        xEventGroupWaitBits(event_group_, PROCESSOR_RUNNING, pdFALSE, pdTRUE, portMAX_DELAY);

        auto res = afe_iface_->fetch_with_delay(afe_data_, portMAX_DELAY);
        if ((xEventGroupGetBits(event_group_) & PROCESSOR_RUNNING) == 0) {
            continue;
        }
        if (res == nullptr || res->ret_value == ESP_FAIL) {
            if (res != nullptr) {
                ESP_LOGI(TAG, "Error code: %d", res->ret_value);
            }
            continue;
        }

        // VAD state change - 优化：只在状态变化时处理缓存
        if (vad_state_change_callback_) {
            // 添加调试日志显示VAD原始状态
            // static int vad_log_counter = 0;
            // if (++vad_log_counter % 10 == 0) {  // 每50次打印一次，避免日志过多
            //     // ESP_LOGI(TAG, "VAD raw state: %d, is_speaking_: %s, cache_size: %d",
            //     //     res->vad_state, is_speaking_ ? "true" : "false", res->vad_cache_size);
            // }
    
            if (res->vad_state == VAD_SPEECH && !is_speaking_) {
                // 🎯 新增：音频能量阈值过滤
                // 计算当前音频帧的能量
                size_t current_samples = res->data_size / sizeof(int16_t);
                float rms_energy = CalculateRMSEnergy(res->data, current_samples);
                current_energy_dbfs_ = ConvertToDBFS(rms_energy);

                // 检查能量是否满足阈值条件
                bool energy_sufficient = CheckEnergyThreshold(current_energy_dbfs_);

                // 只有VAD检测到语音且能量足够时才触发
                if (energy_sufficient) {
                    is_speaking_ = true;//检测到用户说话
                    vad_cache_just_processed_ = false;  // 🎯 重置缓存处理标记
                    ESP_LOGI(TAG, "VAD triggered: energy=%.1fdBFS (smoothed=%.1fdBFS), threshold=%.1fdBFS",
                             current_energy_dbfs_, smoothed_energy_dbfs_, vad_energy_threshold_dbfs_);
                // 1. VAD算法固有延迟：VAD无法在首帧精准触发，可能有1-3帧延迟
                // 2. 防误触机制：需持续触发时间达到vad_min_speech_ms才会正式触发
                if (res->vad_cache_size > 0 && output_callback_) {
                    // 🛡️ 增强安全检查
                    const size_t MAX_CACHE_SIZE = 8192;  // 8KB最大缓存限制

                    if (res->vad_cache == nullptr) {
                        ESP_LOGE(TAG, "VAD cache pointer is null");
                    } else if (res->vad_cache_size % sizeof(int16_t) != 0) {
                        ESP_LOGE(TAG, "VAD cache size not aligned: %d", res->vad_cache_size);
                    } else if (res->vad_cache_size > MAX_CACHE_SIZE) {
                        ESP_LOGE(TAG, "VAD cache size too large: %d > %d", res->vad_cache_size, MAX_CACHE_SIZE);
                    } else {
                        // 安全处理VAD缓存
                        size_t cache_samples = res->vad_cache_size / sizeof(int16_t);
                        int16_t* cache_data = (int16_t*)res->vad_cache;

                        // ESP_LOGI(TAG, "Processing VAD cache: %d bytes, %d samples",
                        //         res->vad_cache_size, (int)cache_samples);

                        try {
                            //检测到新语音时，清空旧缓冲区保证时间顺序
                            output_buffer_.clear();

                            // 预先保留空间，避免多次重新分配
                            output_buffer_.reserve(cache_samples);

                            // 正确顺序：先添加VAD缓存数据（历史数据）
                            output_buffer_.insert(output_buffer_.end(), cache_data, cache_data + cache_samples);

                            // ESP_LOGI(TAG, "VAD cache processed successfully, buffer size: %d",
                            //         (int)output_buffer_.size());
                            vad_cache_just_processed_ = true;  // 🎯 标记已处理VAD缓存
                        } catch (const std::exception& e) {
                            ESP_LOGE(TAG, "Exception processing VAD cache: %s", e.what());
                        } catch (...) {
                            ESP_LOGE(TAG, "Unknown exception processing VAD cache");
                        }
                    }
                }

                    vad_state_change_callback_(true);
                } else {
                    // VAD检测到语音但能量不足，记录但不触发
                    static int low_energy_count = 0;
                    if (++low_energy_count % 10 == 0) {  // 每10次记录一次
                        ESP_LOGD(TAG, "VAD detected but energy insufficient: %.1fdBFS < %.1fdBFS",
                                smoothed_energy_dbfs_, vad_energy_threshold_dbfs_);
                    }
                }
            } else if (res->vad_state == VAD_SILENCE && is_speaking_) {
                is_speaking_ = false;//🎯用户停止说话
                // 重置能量检测状态
                energy_above_threshold_frames_ = 0;
                energy_below_threshold_frames_ = 0;
                ESP_LOGI(TAG, "VAD silence detected, final energy: %.1fdBFS", current_energy_dbfs_);
                vad_state_change_callback_(false);
            }
        }

        if (output_callback_) {
            size_t samples = res->data_size / sizeof(int16_t);

            // 🎯 优化：避免VAD缓存和当前帧重复
            if (vad_cache_just_processed_) {
                // 刚处理了VAD缓存，跳过当前帧避免重复
                ESP_LOGD(TAG, "Skipping current frame (samples=%d) to avoid duplication with VAD cache", (int)samples);
                vad_cache_just_processed_ = false;  // 重置标记
            } else {
                // 正常添加当前帧数据
                output_buffer_.insert(output_buffer_.end(), res->data, res->data + samples);
            }
            
            // Output complete frames when buffer has enough data
            while (output_buffer_.size() >= frame_samples_) {
                if (output_buffer_.size() == frame_samples_) {
                    // If buffer size equals frame size, move the entire buffer
                    output_callback_(std::move(output_buffer_));
                    output_buffer_.clear();
                    output_buffer_.reserve(frame_samples_);
                } else {
                    // If buffer size exceeds frame size, copy one frame and remove it
                    output_callback_(std::vector<int16_t>(output_buffer_.begin(), output_buffer_.begin() + frame_samples_));
                    output_buffer_.erase(output_buffer_.begin(), output_buffer_.begin() + frame_samples_);
                }
            }
        }
    }
}

void AfeAudioProcessor::EnableDeviceAec(bool enable) {
    if (enable) {
#if CONFIG_USE_DEVICE_AEC
        afe_iface_->disable_vad(afe_data_);
        afe_iface_->enable_aec(afe_data_);
#else
        ESP_LOGE(TAG, "Device AEC is not supported");
#endif
    } else {
        afe_iface_->disable_aec(afe_data_);
        afe_iface_->enable_vad(afe_data_);
    }
}

// VAD能量检测辅助函数实现
float AfeAudioProcessor::CalculateRMSEnergy(const int16_t* data, size_t samples) {
    if (!data || samples == 0) {
        return 0.0f;
    }

    double sum_squares = 0.0;
    for (size_t i = 0; i < samples; i++) {
        double sample = static_cast<double>(data[i]) / 32768.0; // 归一化到[-1, 1]
        sum_squares += sample * sample;
    }

    return static_cast<float>(sqrt(sum_squares / samples));
}

float AfeAudioProcessor::ConvertToDBFS(float rms) {
    if (rms <= 0.0f) {
        return -100.0f; // 静音时返回很低的dB值
    }

    // 转换为dBFS (0dBFS = 满量程)
    return 20.0f * log10f(rms);
}

bool AfeAudioProcessor::CheckEnergyThreshold(float energy_dbfs) {
    // 更新平滑后的能量值
    smoothed_energy_dbfs_ = vad_energy_smooth_factor_ * energy_dbfs +
                           (1.0f - vad_energy_smooth_factor_) * smoothed_energy_dbfs_;

    // 检查能量是否超过阈值
    if (smoothed_energy_dbfs_ > vad_energy_threshold_dbfs_) {
        energy_above_threshold_frames_++;
        energy_below_threshold_frames_ = 0;

        // 需要连续几帧超过阈值才认为是真实语音 
        return energy_above_threshold_frames_ >= vad_min_energy_frames_;
    } else {
        energy_below_threshold_frames_++;
        energy_above_threshold_frames_ = 0;

        // 低于阈值时不触发语音（严格门控）
        return false;
    }
}

// VAD能量阈值动态调整方法
void AfeAudioProcessor::SetVadEnergyThreshold(float threshold_dbfs) {
    // 参数有效性检查
    if (threshold_dbfs > 0.0f || threshold_dbfs < -100.0f) {
        ESP_LOGW(TAG, "Invalid VAD energy threshold: %.1fdBFS (valid range: -100.0 to 0.0)", threshold_dbfs);
        return;
    }

    float old_threshold = vad_energy_threshold_dbfs_;
    vad_energy_threshold_dbfs_ = threshold_dbfs;

    // 重置能量检测状态，避免阈值变化时的状态混乱
    energy_above_threshold_frames_ = 0;
    energy_below_threshold_frames_ = 0;

    ESP_LOGI(TAG, "VAD energy threshold changed: %.1fdBFS -> %.1fdBFS",
             old_threshold, threshold_dbfs);
}

float AfeAudioProcessor::GetCurrentEnergyLevel() const {
    return smoothed_energy_dbfs_;
}

float AfeAudioProcessor::GetVadEnergyThreshold() const {
    return vad_energy_threshold_dbfs_;
}

void AfeAudioProcessor::SetVadSmoothFactor(float smooth_factor) {
    // 参数有效性检查
    if (smooth_factor <= 0.0f || smooth_factor >= 1.0f) {
        ESP_LOGW(TAG, "Invalid VAD smooth factor: %.2f (valid range: 0.0 to 1.0)", smooth_factor);
        return;
    }

    float old_factor = vad_energy_smooth_factor_;
    vad_energy_smooth_factor_ = smooth_factor;

    ESP_LOGI(TAG, "VAD smooth factor changed: %.2f -> %.2f", old_factor, smooth_factor);
}

void AfeAudioProcessor::SetVadMinEnergyFrames(int min_frames) {
    // 参数有效性检查
    if (min_frames < 1 || min_frames > 10) {
        ESP_LOGW(TAG, "Invalid VAD min energy frames: %d (valid range: 1 to 10)", min_frames);
        return;
    }

    int old_frames = vad_min_energy_frames_;
    vad_min_energy_frames_ = min_frames;

    // 重置帧计数器
    energy_above_threshold_frames_ = 0;
    energy_below_threshold_frames_ = 0;

    ESP_LOGI(TAG, "VAD min energy frames changed: %d -> %d", old_frames, min_frames);
}
