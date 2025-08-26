#include "qmi8658.h"

static const char TAG[] = "QMI8658";

QMI8658::QMI8658(i2c_master_bus_handle_t i2c_bus, uint8_t addr)
    : I2cDevice(i2c_bus, addr)
    , initialized_(false)
    , last_acc_x_fixed_(0)
    , last_acc_y_fixed_(0)
    , last_acc_z_fixed_(0)
    , first_run_(true) {
}

QMI8658::~QMI8658() {
    // 析构函数，清理资源
}

// 移除RegisterRead和RegisterWriteByte方法，直接使用继承的ReadRegs和WriteReg

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
        WriteReg(QMI8658_RESET, 0xb0);
    } catch (...) {
        ESP_LOGE(TAG, "Failed to reset device");
        return false;
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // 配置寄存器
    try {
        WriteReg(QMI8658_CTRL1, 0x60);  // 地址自动增加
        WriteReg(QMI8658_CTRL7, 0x03);  // 允许加速度和陀螺仪
        WriteReg(QMI8658_CTRL2, 0x15);  // ACC 4g 250Hz
        WriteReg(QMI8658_CTRL3, 0x00);  // GRY 512dps 250Hz
    } catch (...) {
        ESP_LOGE(TAG, "Failed to configure device registers");
        return false;
    }
    
    initialized_ = true;
    ESP_LOGI(TAG, "QMI8658 initialized successfully");
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
    
    // 阈值检查
    motion_level_t motion_level;
    if (total_delta_fixed < 3277) {
        motion_level = MOTION_LEVEL_IDLE;      // 静止
    } else if (total_delta_fixed < 13107) {
        motion_level = MOTION_LEVEL_SLIGHT;    // 轻微运动
    } else if (total_delta_fixed < 26214) {
        motion_level = MOTION_LEVEL_MODERATE;  // 中等运动
    } else {
        motion_level = MOTION_LEVEL_INTENSE;   // 剧烈运动
    }
    
    return motion_level;
}

void QMI8658::CalculateAngles(t_sQMI8658 *data) {
    if (!data) return;
    
    float temp;
    
    // 根据加速度计算倾角值并转换为角度
    temp = (float)data->acc_x / sqrt(((float)data->acc_y * (float)data->acc_y +
                                     (float)data->acc_z * (float)data->acc_z));
    data->AngleX = atan(temp) * 57.29578f;  // 180/π=57.29578
    
    temp = (float)data->acc_y / sqrt(((float)data->acc_x * (float)data->acc_x +
                                     (float)data->acc_z * (float)data->acc_z));
    data->AngleY = atan(temp) * 57.29578f;
    
    temp = sqrt(((float)data->acc_x * (float)data->acc_x +
                 (float)data->acc_y * (float)data->acc_y)) /
           (float)data->acc_z;
    data->AngleZ = atan(temp) * 57.29578f;
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
    
    return true;
}
