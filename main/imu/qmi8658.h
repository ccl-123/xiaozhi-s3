#pragma once

#include "boards/common/i2c_device.h"
#include <stdio.h>
#include "esp_err.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"
#include <string>

/*******************************************************************************/
/***************************  姿态传感器 QMI8658 ↓   ****************************/
#define QMI8658_SENSOR_ADDR 0x6A   // QMI8658 I2C地址

// QMI8658寄存器配置值定义
#define QMI8658_CTRL1_AUTO_INC      0x60  // 地址自动增加
#define QMI8658_CTRL7_ACC_GYR_EN    0x03  // 启用加速度计和陀螺仪
#define QMI8658_CTRL2_ACC_4G_250HZ  0x15  // 加速度计±4g量程，250Hz采样率
#define QMI8658_CTRL3_GYR_512DPS_250HZ 0x54  // 陀螺仪±512dps量程，250Hz采样率
#define QMI8658_RESET_CMD           0xb0  // 软件复位命令

// QMI8658寄存器地址
enum qmi8658_reg {
    QMI8658_WHO_AM_I,
    QMI8658_REVISION_ID,
    QMI8658_CTRL1,
    QMI8658_CTRL2,
    QMI8658_CTRL3,
    QMI8658_CTRL4,
    QMI8658_CTRL5,
    QMI8658_CTRL6,
    QMI8658_CTRL7,
    QMI8658_CTRL8,
    QMI8658_CTRL9,
    QMI8658_CATL1_L,
    QMI8658_CATL1_H,
    QMI8658_CATL2_L,
    QMI8658_CATL2_H,
    QMI8658_CATL3_L,
    QMI8658_CATL3_H,
    QMI8658_CATL4_L,
    QMI8658_CATL4_H,
    QMI8658_FIFO_WTM_TH,
    QMI8658_FIFO_CTRL,
    QMI8658_FIFO_SMPL_CNT,
    QMI8658_FIFO_STATUS,
    QMI8658_FIFO_DATA,
    QMI8658_STATUSINT = 45,
    QMI8658_STATUS0,
    QMI8658_STATUS1,
    QMI8658_TIMESTAMP_LOW,
    QMI8658_TIMESTAMP_MID,
    QMI8658_TIMESTAMP_HIGH,
    QMI8658_TEMP_L,
    QMI8658_TEMP_H,
    QMI8658_AX_L,
    QMI8658_AX_H,
    QMI8658_AY_L,
    QMI8658_AY_H,
    QMI8658_AZ_L,
    QMI8658_AZ_H,
    QMI8658_GX_L,
    QMI8658_GX_H,
    QMI8658_GY_L,
    QMI8658_GY_H,
    QMI8658_GZ_L,
    QMI8658_GZ_H,
    QMI8658_COD_STATUS = 70,
    QMI8658_dQW_L = 73,
    QMI8658_dQW_H,
    QMI8658_dQX_L,
    QMI8658_dQX_H,
    QMI8658_dQY_L,
    QMI8658_dQY_H,
    QMI8658_dQZ_L,
    QMI8658_dQZ_H,
    QMI8658_dVX_L,
    QMI8658_dVX_H,
    QMI8658_dVY_L,
    QMI8658_dVY_H,
    QMI8658_dVZ_L,
    QMI8658_dVZ_H,
    QMI8658_TAP_STATUS = 89,
    QMI8658_STEP_CNT_LOW,
    QMI8658_STEP_CNT_MIDL,
    QMI8658_STEP_CNT_HIGH,
    QMI8658_RESET = 96
};

// IMU数据结构体
typedef struct {
    // 原始数据（LSB值）
    int16_t acc_x = 0;
    int16_t acc_y = 0;
    int16_t acc_z = 0;
    int16_t gyr_x = 0;
    int16_t gyr_y = 0;
    int16_t gyr_z = 0;

    // 转换后的物理单位值
    float acc_x_g = 0.0f;      // 加速度 (g)
    float acc_y_g = 0.0f;
    float acc_z_g = 0.0f;
    float gyr_x_dps = 0.0f;    // 角速度 (°/s)
    float gyr_y_dps = 0.0f;
    float gyr_z_dps = 0.0f;

    // 计算得出的角度和运动状态
    float AngleX = 0.0;
    float AngleY = 0.0;
    float AngleZ = 0.0;
    int motion = 0;
    int fall_state = 0;  // 摔倒检测状态

    std::string ToString() const {
        return std::to_string(acc_x) + " " + std::to_string(acc_y) + " " +
               std::to_string(acc_z) + " " + std::to_string(gyr_x) + " " +
               std::to_string(gyr_y) + " " + std::to_string(gyr_z);
    }
} t_sQMI8658;

// 定义运动等级
typedef enum {
    MOTION_LEVEL_IDLE = 0,    // 静止
    MOTION_LEVEL_SLIGHT = 1,  // 轻微运动
    MOTION_LEVEL_MODERATE = 2,// 中等运动
    MOTION_LEVEL_INTENSE = 3  // 剧烈运动
} motion_level_t;

// 摔倒检测状态
typedef enum {
    FALL_STATE_NORMAL = 0,     // 正常状态
    FALL_STATE_IMPACT = 1,     // 检测到冲击
    FALL_STATE_CONFIRMING = 2, // 确认摔倒中
    FALL_STATE_DETECTED = 3    // 确认摔倒
} fall_detection_state_t;

// 摔倒检测配置参数
typedef struct {
    float acc_threshold;        // 加速度冲击阈值 (g)
    float gyro_threshold;       // 角速度阈值 (°/s)
    float posture_angle_threshold; // 姿态角变化阈值 (°)
    float stable_acc_low;       // 稳定加速度下限 (g)
    float stable_acc_high;      // 稳定加速度上限 (g)
    float stable_gyro;          // 稳定角速度阈值 (°/s)
    uint32_t stable_time_ms;    // 确认时间 (毫秒)
} fall_detection_config_t;

// QMI8658 IMU传感器类 - 继承I2cDevice以访问protected方法
class QMI8658 : public I2cDevice {
private:
    bool initialized_;

    // 运动检测相关
    int32_t last_acc_x_fixed_;
    int32_t last_acc_y_fixed_;
    int32_t last_acc_z_fixed_;
    bool first_run_;

    // 陀螺仪零偏校准
    float gyr_offset_x_;
    float gyr_offset_y_;
    float gyr_offset_z_;
    bool calibrated_;

    // 摔倒检测相关
    fall_detection_config_t fall_config_;
    fall_detection_state_t fall_state_;
    uint64_t stable_start_time_;
    bool possible_fall_;

    // 内部函数
    motion_level_t DetectMotion(t_sQMI8658 *p);
    void ConvertToPhysicalUnits(t_sQMI8658 *data);

public:
    QMI8658(i2c_master_bus_handle_t i2c_bus, uint8_t addr);
    ~QMI8658();

    // 初始化和配置
    bool Initialize();
    bool IsInitialized() const { return initialized_; }

    // 陀螺仪校准
    void CalibrateGyroscope();
    bool IsCalibrated() const { return calibrated_; }
    
    // 数据读取
    bool ReadAccAndGyr(t_sQMI8658 *data);
    bool ReadMotionData(t_sQMI8658 *data);
    
    // 角度计算
    void CalculateAngles(t_sQMI8658 *data);
    
    // 运动检测
    motion_level_t GetMotionLevel(t_sQMI8658 *data);

    // 摔倒检测
    void InitializeFallDetection();
    fall_detection_state_t DetectFall(t_sQMI8658 *data);
    void SetFallDetectionConfig(const fall_detection_config_t& config);
    fall_detection_state_t GetFallState() const { return fall_state_; }
};

/***************************  姿态传感器 QMI8658 ↑  ****************************/
/*******************************************************************************/
