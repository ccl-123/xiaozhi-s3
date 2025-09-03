#include "qmi8658.h"

static const char TAG[] = "QMI8658";

QMI8658::QMI8658(i2c_master_bus_handle_t i2c_bus, uint8_t addr)
    : I2cDevice(i2c_bus, addr)
    , initialized_(false)
    , last_acc_x_fixed_(0)
    , last_acc_y_fixed_(0)
    , last_acc_z_fixed_(0)
    , first_run_(true)
    , gyr_offset_x_(0.0f)
    , gyr_offset_y_(0.0f)
    , gyr_offset_z_(0.0f)
    , calibrated_(false)
    , fall_state_(FALL_STATE_NORMAL)
    , stable_start_time_(0)
    , possible_fall_(false)
    , impact_time_(0)
    , max_gyro_magnitude_(0.0f)
    , filtered_angle_x_(0.0f)
    , filtered_angle_y_(0.0f)
    , filtered_angle_z_(0.0f)
    , last_update_time_(0)
    , complementary_alpha_(0.98f) {
    InitializeFallDetection();
}

QMI8658::~QMI8658() {
    // 析构函数，清理资源
}


bool QMI8658::Initialize() {
    
    // 读取设备ID验证
    uint8_t id = 0;
    try {
        ReadRegs(QMI8658_WHO_AM_I, &id, 1);
    } catch (...) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register");
        return false;
    }

    // 验证设备ID
    int retry_count = 0;
    while (id != 0x05 && retry_count < 10) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
        try {
            ReadRegs(QMI8658_WHO_AM_I, &id, 1);
        } catch (...) {
            ESP_LOGE(TAG, "Failed to read WHO_AM_I register on retry %d", retry_count);
            return false;
        }
        retry_count++;
    }
    
    if (id != 0x05) {
        ESP_LOGE(TAG, "Invalid device ID: 0x%02X, expected 0x05", id);
        return false;
    }
    
    ESP_LOGI(TAG, "QMI8658 device detected successfully!");
    
    // 软件复位
    try {
        WriteReg(QMI8658_RESET, QMI8658_RESET_CMD);
    } catch (...) {
        ESP_LOGE(TAG, "Failed to reset device");
        return false;
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // 配置寄存器
    try {
        WriteReg(QMI8658_CTRL1, QMI8658_CTRL1_AUTO_INC);      // 地址自动增加
        WriteReg(QMI8658_CTRL7, QMI8658_CTRL7_ACC_GYR_EN);   // 启用加速度计和陀螺仪
        WriteReg(QMI8658_CTRL2, QMI8658_CTRL2_ACC_4G_250HZ); // 加速度计±4g，250Hz
        WriteReg(QMI8658_CTRL3, QMI8658_CTRL3_GYR_512DPS_250HZ); // 陀螺仪±512dps，250Hz
    } catch (...) {
        ESP_LOGE(TAG, "Failed to configure device registers");
        return false;
    }
    
    initialized_ = true;
    ESP_LOGI(TAG, "QMI8658 initialized successfully");

    // 初始化完成后进行陀螺仪校准
    ESP_LOGI(TAG, "Starting gyroscope calibration...");
    CalibrateGyroscope();

    return true;
}

bool QMI8658::ReadAccAndGyr(t_sQMI8658 *data) {
    if (!initialized_ || !data) {
        return false;
    }
    
    uint8_t status;
    try {
        ReadRegs(QMI8658_STATUS0, &status, 1);
    } catch (...) {
        return false;
    }

    // 检查数据是否就绪
    if (!(status & 0x03)) {
        return false;  // 数据未就绪
    }

    // 读取6轴数据 (12字节)
    int16_t buf[6];
    try {
        ReadRegs(QMI8658_AX_L, (uint8_t*)buf, 12);
    } catch (...) {
        return false;
    }
    
    data->acc_x = buf[0];
    data->acc_y = buf[1];
    data->acc_z = buf[2];
    data->gyr_x = buf[3];
    data->gyr_y = buf[4];
    data->gyr_z = buf[5];

    // 转换为物理单位
    ConvertToPhysicalUnits(data);

    return true;
}

motion_level_t QMI8658::DetectMotion(t_sQMI8658 *p) {
    // 数据有效性检查
    if (abs(p->acc_x) > 32768 || abs(p->acc_y) > 32768 || abs(p->acc_z) > 32768) {
        return MOTION_LEVEL_IDLE;
    }
    
    // 使用定点数计算代替浮点数计算
    static const int32_t LSB_TO_G_FIXED = 8;  // 1/8192 in Q16.16
    
    // 当前加速度转换为定点数表示 (g单位)
    int32_t curr_acc_x_fixed = (p->acc_x * LSB_TO_G_FIXED);
    int32_t curr_acc_y_fixed = (p->acc_y * LSB_TO_G_FIXED);
    int32_t curr_acc_z_fixed = (p->acc_z * LSB_TO_G_FIXED);
    
    // 首次运行时，初始化历史值为当前值
    if (first_run_) {
        last_acc_x_fixed_ = curr_acc_x_fixed;
        last_acc_y_fixed_ = curr_acc_y_fixed;
        last_acc_z_fixed_ = curr_acc_z_fixed;
        first_run_ = false;
        return MOTION_LEVEL_IDLE; // 首次运行返回静止状态
    }
    
    // 计算变化量
    int32_t delta_x_fixed = abs(curr_acc_x_fixed - last_acc_x_fixed_);
    int32_t delta_y_fixed = abs(curr_acc_y_fixed - last_acc_y_fixed_);
    int32_t delta_z_fixed = abs(curr_acc_z_fixed - last_acc_z_fixed_);
    
    // 更新历史值
    last_acc_x_fixed_ = curr_acc_x_fixed;
    last_acc_y_fixed_ = curr_acc_y_fixed;
    last_acc_z_fixed_ = curr_acc_z_fixed;
    
    // 计算总变化量
    int32_t total_delta_fixed = delta_x_fixed + delta_y_fixed + delta_z_fixed;
    
    // 阈值检查（将差分归一化到20ms基准，不改原阈值）
    // 当前采样周期约为 4ms (250Hz)，后续如变更请调整 kDtActualMs
    constexpr float kDtBaseMs   = 20.0f;
    constexpr float kDtActualMs = 4.0f;
    int32_t norm = static_cast<int32_t>((kDtBaseMs / kDtActualMs) * static_cast<float>(total_delta_fixed));

    motion_level_t motion_level;
    if (norm < 3277) {
        motion_level = MOTION_LEVEL_IDLE;      // 静止
    } else if (norm < 13107) {
        motion_level = MOTION_LEVEL_SLIGHT;    // 轻微运动
    } else if (norm < 26214) {
        motion_level = MOTION_LEVEL_MODERATE;  // 中等运动
    } else {
        motion_level = MOTION_LEVEL_INTENSE;   // 剧烈运动
    }

    return motion_level;
}

void QMI8658::CalculateAngles(t_sQMI8658 *data) {
    if (!data) return;

    uint64_t current_time = esp_timer_get_time() / 1000; // 毫秒
    if (last_update_time_ == 0) {
        // 首次初始化：使用加速度计计算初始角度
        filtered_angle_x_ = atan2(data->acc_y_g,
                                 sqrt(data->acc_x_g * data->acc_x_g + data->acc_z_g * data->acc_z_g)) * 57.29578f;
        filtered_angle_y_ = atan2(-data->acc_x_g,
                                 sqrt(data->acc_y_g * data->acc_y_g + data->acc_z_g * data->acc_z_g)) * 57.29578f;
        filtered_angle_z_ = 0.0f; // yaw角初始化为0
        last_update_time_ = current_time;

    } else {
        // 计算时间间隔
        float dt = (current_time - last_update_time_) / 1000.0f; // 转换为秒

        // 限制dt范围，防止异常值
        if (dt > 0.1f) dt = 0.1f; // 最大100ms
        if (dt < 0.001f) dt = 0.001f; // 最小1ms

        // 步骤1：陀螺仪积分计算角度变化（短期精准）
        float gyro_angle_x = filtered_angle_x_ + data->gyr_x_dps * dt;
        float gyro_angle_y = filtered_angle_y_ + data->gyr_y_dps * dt;
        float gyro_angle_z = filtered_angle_z_ + data->gyr_z_dps * dt;

        // 步骤2：加速度计计算当前角度（长期稳定）
        float accel_angle_x = atan2(data->acc_y_g,
                                   sqrt(data->acc_x_g * data->acc_x_g + data->acc_z_g * data->acc_z_g)) * 57.29578f;
        float accel_angle_y = atan2(-data->acc_x_g,
                                   sqrt(data->acc_y_g * data->acc_y_g + data->acc_z_g * data->acc_z_g)) * 57.29578f;

        // 步骤3：互补滤波融合
        // Angle = α * (Angle + GyroRate * dt) + (1-α) * AccelAngle
        filtered_angle_x_ = complementary_alpha_ * gyro_angle_x + (1.0f - complementary_alpha_) * accel_angle_x;
        filtered_angle_y_ = complementary_alpha_ * gyro_angle_y + (1.0f - complementary_alpha_) * accel_angle_y;
        filtered_angle_z_ = gyro_angle_z; // yaw角只能通过陀螺仪积分（需要磁力计才能修正）

        // 角度范围限制
        if (filtered_angle_x_ > 180.0f) filtered_angle_x_ -= 360.0f;
        if (filtered_angle_x_ < -180.0f) filtered_angle_x_ += 360.0f;
        if (filtered_angle_y_ > 180.0f) filtered_angle_y_ -= 360.0f;
        if (filtered_angle_y_ < -180.0f) filtered_angle_y_ += 360.0f;
        if (filtered_angle_z_ > 180.0f) filtered_angle_z_ -= 360.0f;
        if (filtered_angle_z_ < -180.0f) filtered_angle_z_ += 360.0f;

        last_update_time_ = current_time;
    }

    data->AngleX = filtered_angle_x_;
    data->AngleY = filtered_angle_y_;
    data->AngleZ = filtered_angle_z_;

    // 角度数据四舍五入到4位小数
    data->AngleX = roundf(data->AngleX * 10000.0f) / 10000.0f;
    data->AngleY = roundf(data->AngleY * 10000.0f) / 10000.0f;
    data->AngleZ = roundf(data->AngleZ * 10000.0f) / 10000.0f;
}

motion_level_t QMI8658::GetMotionLevel(t_sQMI8658 *data) {
    if (!data) return MOTION_LEVEL_IDLE;
    return DetectMotion(data);
}

bool QMI8658::ReadMotionData(t_sQMI8658 *data) {
    if (!ReadAccAndGyr(data)) {
        return false;
    }

    // 检测运动强度
    motion_level_t motion = DetectMotion(data);
    data->motion = static_cast<int>(motion);

    // 计算倾角
    CalculateAngles(data);

    // 摔倒检测
    fall_detection_state_t fall_state = DetectFall(data);
    data->fall_state = static_cast<int>(fall_state);

    return true;
}

void QMI8658::ConvertToPhysicalUnits(t_sQMI8658 *data) {
    if (!data) return;

    // 换算系数定义
    const float ACC_LSB_TO_G = 1.0f / 8192.0f;      // ±4g量程，16位ADC
    const float GYR_LSB_TO_DPS = 1.0f / 64.0f;      // ±512dps量程，16位ADC

    // 转换加速度计数据到g单位
    data->acc_x_g = data->acc_x * ACC_LSB_TO_G;
    data->acc_y_g = data->acc_y * ACC_LSB_TO_G;
    data->acc_z_g = data->acc_z * ACC_LSB_TO_G;

    // 转换陀螺仪数据到°/s单位
    data->gyr_x_dps = data->gyr_x * GYR_LSB_TO_DPS;
    data->gyr_y_dps = data->gyr_y * GYR_LSB_TO_DPS;
    data->gyr_z_dps = data->gyr_z * GYR_LSB_TO_DPS;

    // 应用零偏补偿
    if (calibrated_) {
        data->gyr_x_dps -= gyr_offset_x_;
        data->gyr_y_dps -= gyr_offset_y_;
        data->gyr_z_dps -= gyr_offset_z_;
    }

    // 设置 1°/s 死区，抑制微小噪声
    const float GYR_DEADBAND = 1.0f;
    if (fabsf(data->gyr_x_dps) < GYR_DEADBAND) data->gyr_x_dps = 0.0f;
    if (fabsf(data->gyr_y_dps) < GYR_DEADBAND) data->gyr_y_dps = 0.0f;
    if (fabsf(data->gyr_z_dps) < GYR_DEADBAND) data->gyr_z_dps = 0.0f;

    // 四舍五入到4位小数
    data->acc_x_g = roundf(data->acc_x_g * 10000.0f) / 10000.0f;
    data->acc_y_g = roundf(data->acc_y_g * 10000.0f) / 10000.0f;
    data->acc_z_g = roundf(data->acc_z_g * 10000.0f) / 10000.0f;
    data->gyr_x_dps = roundf(data->gyr_x_dps * 10000.0f) / 10000.0f;
    data->gyr_y_dps = roundf(data->gyr_y_dps * 10000.0f) / 10000.0f;
    data->gyr_z_dps = roundf(data->gyr_z_dps * 10000.0f) / 10000.0f;
}

void QMI8658::InitializeFallDetection() {
    // 设置默认的摔倒检测参数
    fall_config_.acc_threshold = 2.5f;        // 2.5g 冲击阈值
    fall_config_.gyro_threshold = 150.0f;     // 150°/s 角速度阈值
    fall_config_.posture_angle_threshold = 40.0f; // 40° 姿态变化阈值
    fall_config_.stable_acc_low = 0.8f;       // 0.8g 稳定下限
    fall_config_.stable_acc_high = 1.2f;      // 1.2g 稳定上限
    fall_config_.stable_gyro = 20.0f;         // 20°/s 稳定角速度阈值
    fall_config_.stable_time_ms = 1000;       // 1秒确认时间

    fall_state_ = FALL_STATE_NORMAL;
    stable_start_time_ = 0;
    possible_fall_ = false;

    ESP_LOGI(TAG, "Fall detection initialized with default parameters");
}

void QMI8658::SetFallDetectionConfig(const fall_detection_config_t& config) {
    fall_config_ = config;
    ESP_LOGI(TAG, "Fall detection parameters updated");
}

fall_detection_state_t QMI8658::DetectFall(t_sQMI8658 *data) {
    if (!data || !initialized_) {
        return FALL_STATE_NORMAL;
    }

    // 计算加速度和角速度的模长
    float acc_mag = sqrt(data->acc_x_g * data->acc_x_g +
                        data->acc_y_g * data->acc_y_g +
                        data->acc_z_g * data->acc_z_g);

    float gyro_mag = sqrt(data->gyr_x_dps * data->gyr_x_dps +
                         data->gyr_y_dps * data->gyr_y_dps +
                         data->gyr_z_dps * data->gyr_z_dps);

    // 获取当前时间（毫秒）
    uint64_t current_time = esp_timer_get_time() / 1000;

    // 阶段1：检测强冲击 + 大角速度
    if (acc_mag > fall_config_.acc_threshold && gyro_mag > fall_config_.gyro_threshold) {
        possible_fall_ = true;
        stable_start_time_ = 0;  // 重置稳定计时
        impact_time_ = current_time;  // 记录冲击时间
        fall_state_ = FALL_STATE_IMPACT;

        // 重置最大角速度记录
        max_gyro_magnitude_ = 0.0f;

    }

    // 阶段2：摔倒确认逻辑
    if (possible_fall_) {
        // 检查冲击后的时间窗口（1.5秒内必须完成姿态变化）
        if (current_time - impact_time_ > 1500) {
            possible_fall_ = false;
            fall_state_ = FALL_STATE_NORMAL;
            return fall_state_;
        }


        // 记录冲击后的最大角速度（用于判断运动是否足够剧烈）
        float current_gyro_mag = sqrt(data->gyr_x_dps * data->gyr_x_dps +
                                     data->gyr_y_dps * data->gyr_y_dps +
                                     data->gyr_z_dps * data->gyr_z_dps);

        if (current_gyro_mag > max_gyro_magnitude_) {
            max_gyro_magnitude_ = current_gyro_mag;
            ESP_LOGD(TAG, "Updated max_gyro_rate: %.1f°/s (gx=%.1f, gy=%.1f, gz=%.1f)",
                    current_gyro_mag, data->gyr_x_dps, data->gyr_y_dps, data->gyr_z_dps);
        }

        // 检查姿态角变化（使用pitch和roll，即AngleX和AngleY）
        bool posture_changed = (fabs(data->AngleX) > fall_config_.posture_angle_threshold ||
                               fabs(data->AngleY) > fall_config_.posture_angle_threshold);

        // 检查加速度和角速度是否稳定
        bool acc_stable = (acc_mag > fall_config_.stable_acc_low &&
                          acc_mag < fall_config_.stable_acc_high);
        bool gyro_stable = (gyro_mag < fall_config_.stable_gyro);

        // 🎯 角速度检查（必须足够快才算摔倒）
        bool gyro_fast_enough = (max_gyro_magnitude_ > 100.0f); // 至少100°/s的角速度

        if (posture_changed && acc_stable && gyro_stable && gyro_fast_enough) {
            fall_state_ = FALL_STATE_CONFIRMING;

            if (stable_start_time_ == 0) {
                stable_start_time_ = current_time;  // 开始计时
                ESP_LOGW(TAG, "Fall confirmation started. Posture: X=%.1f°, Y=%.1f°, max_gyro=%.1f°/s",
                         data->AngleX, data->AngleY, max_gyro_magnitude_);
            } else if (current_time - stable_start_time_ > fall_config_.stable_time_ms) {
                // 确认摔倒
                fall_state_ = FALL_STATE_DETECTED;
                possible_fall_ = false;  // 重置状态
                stable_start_time_ = current_time;  // 🎯 记录进入DETECTED状态的时间
                ESP_LOGW(TAG, "🚨🚨FALL DETECTED! Final posture: X=%.1f°, Y=%.1f°, max_gyro=%.1f°/s🚨🚨",
                         data->AngleX, data->AngleY, max_gyro_magnitude_);
                // 🎯 不要立即返回，让状态继续处理，确保Application层能捕获到DETECTED状态
            }
        } else {
            // 条件不满足，重置稳定计时
            stable_start_time_ = 0;
            if (fall_state_ == FALL_STATE_CONFIRMING) {
                fall_state_ = FALL_STATE_IMPACT;  // 回到冲击状态
            }
        }

        // 超时重置（防止长时间停留在检测状态）
        if (stable_start_time_ != 0 && current_time - stable_start_time_ > 5000) {  // 5秒超时
            possible_fall_ = false;
            fall_state_ = FALL_STATE_NORMAL;
            ESP_LOGW(TAG, "Fall detection timeout, reset to normal");
        }
    }

    // DETECTED状态自动重置逻辑
    if (fall_state_ == FALL_STATE_DETECTED) {
        static uint64_t recovery_start_time = 0;

        // 检查设备是否已恢复正常姿态
        bool posture_normal = (fabs(data->AngleX) < 30.0f && fabs(data->AngleY) < 30.0f);
        bool movement_stable = (acc_mag > 0.8f && acc_mag < 1.2f && gyro_mag < 30.0f);

        if (posture_normal && movement_stable) {
            // 设备姿态恢复正常且运动稳定，检查持续时间
            if (recovery_start_time == 0) {
                recovery_start_time = current_time;
            } else if (current_time - recovery_start_time > 1500) {  // 1.5秒确认时间
                fall_state_ = FALL_STATE_NORMAL;
                recovery_start_time = 0;
                stable_start_time_ = 0;
                ESP_LOGI(TAG, "✅ Fall state reset to NORMAL - device recovered");
            }
        } else {
            // 重置恢复计时器
            recovery_start_time = 0;
        }

        // 强制超时重置：DETECTED状态超过15秒自动重置
        if (stable_start_time_ != 0 && current_time - stable_start_time_ > 15000) {
            fall_state_ = FALL_STATE_NORMAL;
            stable_start_time_ = 0;
            ESP_LOGW(TAG, "⏰ Fall state force reset after 30s timeout");
        }
    }

    return fall_state_;
}

void QMI8658::CalibrateGyroscope() {
    if (!initialized_) {
        ESP_LOGE(TAG, "Device not initialized, cannot calibrate");
        return;
    }

    ESP_LOGI(TAG, "Calibrating gyroscope... Please keep device stationary for 3 seconds");

    const int calibration_samples = 100;  // 采集100个样本
    float sum_x = 0, sum_y = 0, sum_z = 0;
    int valid_samples = 0;

    for (int i = 0; i < calibration_samples; i++) {
        t_sQMI8658 data;
        if (ReadAccAndGyr(&data)) {
            sum_x += data.gyr_x;
            sum_y += data.gyr_y;
            sum_z += data.gyr_z;
            valid_samples++;
        }
        vTaskDelay(30 / portTICK_PERIOD_MS);  // 30ms间隔
    }

    if (valid_samples > 50) {  // 至少需要50个有效样本
        // 计算平均偏移值（原始数据）
        float avg_x = sum_x / valid_samples;
        float avg_y = sum_y / valid_samples;
        float avg_z = sum_z / valid_samples;

        // 转换为物理单位的偏移值
        const float GYR_LSB_TO_DPS = 1.0f / 64.0f;
        gyr_offset_x_ = avg_x * GYR_LSB_TO_DPS;
        gyr_offset_y_ = avg_y * GYR_LSB_TO_DPS;
        gyr_offset_z_ = avg_z * GYR_LSB_TO_DPS;

        calibrated_ = true;

        ESP_LOGI(TAG, "Gyroscope calibration completed");
        ESP_LOGI(TAG, "Offsets: X=%.2f°/s, Y=%.2f°/s, Z=%.2f°/s",
                 gyr_offset_x_, gyr_offset_y_, gyr_offset_z_);
    } else {
        ESP_LOGE(TAG, "Gyroscope calibration failed: insufficient valid samples (%d)", valid_samples);
        calibrated_ = false;
    }
}
