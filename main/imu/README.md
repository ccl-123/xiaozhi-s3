# QMI8658 IMU传感器模块

## 📋 概述

本模块实现了QMI8658A 6轴IMU传感器的完整驱动，支持3轴加速度计和3轴陀螺仪数据读取、角度计算、运动检测等功能。

## 🔧 硬件规格

### 传感器参数
- **型号**: QMI8658A
- **制造商**: QST (Qst Corporation)
- **类型**: 6轴IMU（3轴加速度计 + 3轴陀螺仪）
- **通信接口**: I2C
- **I2C地址**: 0x6A
- **工作电压**: 1.8V-3.6V
- **数据精度**: 16位ADC

### 测量范围
- **加速度计**: ±4g
- **陀螺仪**: ±512dps (度/秒)
- **采样率**: 250Hz

### 引脚配置
```
SDA: GPIO_NUM_1
SCL: GPIO_NUM_2
频率: 100kHz
```

## 📊 数据结构

### t_sQMI8658 结构体
```cpp
typedef struct {
    int16_t acc_x, acc_y, acc_z;    // 加速度计原始数据 (LSB)
    int16_t gyr_x, gyr_y, gyr_z;    // 陀螺仪原始数据 (LSB)
    float AngleX, AngleY, AngleZ;   // 计算的倾角 (度)
    int motion;                     // 运动强度等级 (0-3)
    
    std::string ToString() const;   // 数据转字符串
} t_sQMI8658;
```

### 运动等级定义
```cpp
typedef enum {
    MOTION_LEVEL_IDLE = 0,      // 静止
    MOTION_LEVEL_SLIGHT = 1,    // 轻微运动
    MOTION_LEVEL_MODERATE = 2,  // 中等运动
    MOTION_LEVEL_INTENSE = 3    // 剧烈运动
} motion_level_t;
```

## 🎯 API接口

### QMI8658 类
```cpp
class QMI8658 : public I2cDevice {
public:
    // 构造函数
    QMI8658(i2c_master_bus_handle_t i2c_bus, uint8_t addr);
    
    // 初始化传感器
    bool Initialize();
    
    // 检查初始化状态
    bool IsInitialized() const;
    
    // 读取6轴原始数据
    bool ReadAccAndGyr(t_sQMI8658 *data);
    
    // 读取完整运动数据（包含角度和运动检测）
    bool ReadMotionData(t_sQMI8658 *data);
    
    // 计算倾角
    void CalculateAngles(t_sQMI8658 *data);
    
    // 获取运动等级
    motion_level_t GetMotionLevel(t_sQMI8658 *data);
};
```

## 📈 数据换算

### 换算公式
```cpp
// 加速度计：LSB → g
float acc_g = raw_value / 8192.0f;

// 陀螺仪：LSB → 度/秒
float gyr_dps = raw_value / 64.0f;

// 角度计算（基于重力分量）
float angle_x = atan(acc_x / sqrt(acc_y² + acc_z²)) * 57.29578f;
```

### 典型数值
- **静止状态**: acc_z ≈ ±8192 LSB (1g重力)
- **水平放置**: acc_x, acc_y ≈ 0 LSB
- **无旋转**: gyr_x, gyr_y, gyr_z ≈ 0 LSB

## 🔍 运动检测算法

### 检测原理
基于加速度变化量的运动强度检测：
1. 计算当前与历史加速度的差值
2. 使用定点数运算避免浮点计算
3. 多级阈值判断运动强度
4. 防抖处理避免误触发

### 阈值设置
```cpp
// 变化量阈值（定点数表示）
静止:     < 3277   (约0.05g变化)
轻微:   3277-13107  (0.05g-0.2g)
中等:  13107-26214  (0.2g-0.4g)
剧烈:     > 26214   (>0.4g)
```

## 📡 数据输出格式

### 控制台日志（每0.5秒）
```
I Application: === IMU Data ===
I Application: Accelerometer: X=-0.0017g, Y=-0.0052g, Z=1.0104g
I Application: Gyroscope: X=-54.6719°/s, Y=132.5469°/s, Z=-20.8438°/s
I Application: Angles: X=-0.1000°, Y=-0.3100°, Z=0.3200°
I Application: Motion Level: 0 (IDLE)
I Application: ===============
```

### MQTT数据（每0.5秒）
```json
{
  "imu_type": 0,
  "touch_value": 0,
  "acc_x": -0.0017,      // g单位，4位小数
  "acc_y": -0.0052,      // g单位，4位小数
  "acc_z": 1.0104,       // g单位，4位小数
  "gyr_x": -54.6719,     // 度/秒，4位小数
  "gyr_y": 132.5469,     // 度/秒，4位小数
  "gyr_z": -20.8438,     // 度/秒，4位小数
  "angle_x": -0.1000,    // 度，4位小数
  "angle_y": -0.3100,    // 度，4位小数
  "angle_z": 0.3200,     // 度，4位小数
  "device_id": "167826228953648000"
}
```

## 🚀 使用示例

### 基本使用
```cpp
#include "qmi8658.h"

// 创建IMU实例
QMI8658 imu(i2c_bus, 0x6A);

// 初始化
if (!imu.Initialize()) {
    ESP_LOGE(TAG, "IMU initialization failed");
    return;
}

// 读取数据
t_sQMI8658 data;
if (imu.ReadMotionData(&data)) {
    ESP_LOGI(TAG, "Acc: %.4fg, %.4fg, %.4fg", 
             data.acc_x/8192.0f, data.acc_y/8192.0f, data.acc_z/8192.0f);
    ESP_LOGI(TAG, "Motion level: %d", data.motion);
}
```

### 定时读取
```cpp
// 创建定时器，每100ms读取一次
esp_timer_create_args_t timer_args = {
    .callback = [](void* arg) {
        QMI8658* imu = static_cast<QMI8658*>(arg);
        t_sQMI8658 data;
        if (imu->ReadMotionData(&data)) {
            // 处理数据
        }
    },
    .arg = &imu,
    .name = "imu_timer"
};

esp_timer_handle_t timer;
esp_timer_create(&timer_args, &timer);
esp_timer_start_periodic(timer, 100000); // 100ms
```

## ⚠️ 注意事项

### 硬件要求
1. 确保I2C引脚连接正确（SDA=GPIO1, SCL=GPIO2）
2. 检查传感器电源供应稳定（1.8V-3.6V）
3. 避免I2C总线冲突

### 软件配置
1. 在`config.h`中启用IMU：`#define IMU_ENABLE 1`
2. 确保I2C总线已正确初始化
3. 传感器需要在板级配置中正确集成

### 性能优化
1. 使用定点数计算避免浮点运算开销
2. 合理设置采样频率平衡性能和功耗
3. 运动检测阈值可根据应用场景调整

## 🔧 故障排除

### 常见问题
1. **设备检测失败**: 检查I2C连接和地址
2. **数据读取异常**: 验证传感器初始化状态
3. **运动检测不准确**: 调整阈值或校准传感器

### 调试方法
```bash
# 查看详细日志
idf.py monitor --print_filter="*:I"

# 只看IMU相关日志
idf.py monitor | grep -E "(QMI8658|IMU|Application.*===)"
```

## 📝 版本历史

- **v1.0**: 基础驱动实现，支持数据读取和运动检测
- **v1.1**: 优化数据格式，使用物理单位和4位小数精度
- **v1.2**: 集成MQTT发送，支持实时数据传输

## 📞 技术支持

如需技术支持或报告问题，请参考：
- 硬件手册：QMI8658A数据手册
- 软件文档：ESP-IDF I2C驱动文档
- 项目仓库：xiaozhi-afe IMU模块
