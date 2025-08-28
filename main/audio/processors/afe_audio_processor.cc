#include "afe_audio_processor.h"
#include <esp_log.h>

#define PROCESSOR_RUNNING 0x01

#define TAG "AfeAudioProcessor"

AfeAudioProcessor::AfeAudioProcessor()
    : afe_data_(nullptr) {
    event_group_ = xEventGroupCreate();
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
    afe_config->vad_mode = VAD_MODE_1;  // 官方推荐模式，数值越大触发概率越高
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
            static int vad_log_counter = 0;
            if (++vad_log_counter % 50 == 0) {  // 每50次打印一次，避免日志过多
                ESP_LOGD(TAG, "VAD raw state: %d, is_speaking_: %s, cache_size: %d",
                    res->vad_state, is_speaking_ ? "true" : "false", res->vad_cache_size);
            }

            if (res->vad_state == VAD_SPEECH && !is_speaking_) {
                is_speaking_ = true;//检测到用户说话
                // 1. VAD算法固有延迟：VAD无法在首帧精准触发，可能有1-3帧延迟
                // 2. 防误触机制：需持续触发时间达到vad_min_speech_ms才会正式触发
                if (res->vad_cache_size > 0 && output_callback_) {
                    if (res->vad_cache != nullptr && res->vad_cache_size % sizeof(int16_t) == 0) {
                        ESP_LOGI(TAG, "VAD triggered with cache: %d bytes, processing cached audio", res->vad_cache_size);
                        // 将VAD缓存数据转换为int16_t并加入输出缓冲区开头
                        size_t cache_samples = res->vad_cache_size / sizeof(int16_t);
                        int16_t* cache_data = (int16_t*)res->vad_cache;

                        // 优先处理缓存数据，避免首字截断
                        output_buffer_.insert(output_buffer_.begin(), cache_data, cache_data + cache_samples);

                        ESP_LOGD(TAG, "VAD cache processed: %zu samples (%.1fms audio) added to buffer",
                                 cache_samples, (float)cache_samples / 16.0f);  // 16kHz采样率
                    } else {
                        ESP_LOGW(TAG, "Invalid VAD cache data: ptr=%p, size=%d", res->vad_cache, res->vad_cache_size);
                    }
                }

                vad_state_change_callback_(true);
            } else if (res->vad_state == VAD_SILENCE && is_speaking_) {
                is_speaking_ = false;//🎯 用户停止说话
                vad_state_change_callback_(false);
            }
        }

        if (output_callback_) {
            size_t samples = res->data_size / sizeof(int16_t);
            
            // Add data to buffer
            output_buffer_.insert(output_buffer_.end(), res->data, res->data + samples);
            
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
