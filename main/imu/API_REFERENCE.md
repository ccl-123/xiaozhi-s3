# QMI8658 API参考文档

## 📚 类和接口

### QMI8658 类

#### 构造函数
```cpp
QMI8658(i2c_master_bus_handle_t i2c_bus, uint8_t addr);
```
**参数:**
- `i2c_bus`: I2C总线句柄
- `addr`: I2C设备地址（通常为0x6A）

**示例:**
```cpp
QMI8658 imu(i2c_bus, 0x6A);
```

#### Initialize()
```cpp
bool Initialize();
```
**功能:** 初始化QMI8658传感器
**返回值:** 
- `true`: 初始化成功
- `false`: 初始化失败

**初始化流程:**
1. 读取WHO_AM_I寄存器验证设备ID (0x05)
2. 软件复位传感器
3. 配置控制寄存器
4. 设置采样率和量程

**示例:**
```cpp
if (!imu.Initialize()) {
    ESP_LOGE(TAG, "IMU initialization failed");
}
```

#### IsInitialized()
```cpp
bool IsInitialized() const;
```
**功能:** 检查传感器是否已初始化
**返回值:** 初始化状态

#### ReadAccAndGyr()
```cpp
bool ReadAccAndGyr(t_sQMI8658 *data);
```
**功能:** 读取6轴原始数据
**参数:** `data` - 数据结构指针
**返回值:** 读取成功状态

**读取的数据:**
- `acc_x, acc_y, acc_z`: 加速度计原始值 (LSB)
- `gyr_x, gyr_y, gyr_z`: 陀螺仪原始值 (LSB)

#### ReadMotionData()
```cpp
bool ReadMotionData(t_sQMI8658 *data);
```
**功能:** 读取完整运动数据
**参数:** `data` - 数据结构指针
**返回值:** 读取成功状态

**包含的处理:**
1. 读取6轴原始数据
2. 运动检测分析
3. 角度计算

#### CalculateAngles()
```cpp
void CalculateAngles(t_sQMI8658 *data);
```
**功能:** 计算倾角
**参数:** `data` - 包含加速度数据的结构体

**计算公式:**
```cpp
AngleX = atan(acc_x / sqrt(acc_y² + acc_z²)) * 57.29578°
AngleY = atan(acc_y / sqrt(acc_x² + acc_z²)) * 57.29578°
AngleZ = atan(sqrt(acc_x² + acc_y²) / acc_z) * 57.29578°
```

#### GetMotionLevel()
```cpp
motion_level_t GetMotionLevel(t_sQMI8658 *data);
```
**功能:** 获取运动强度等级
**参数:** `data` - IMU数据结构
**返回值:** 运动等级枚举值

## 📊 数据结构

### t_sQMI8658
```cpp
typedef struct {
    int16_t acc_x = 0;      // X轴加速度 (LSB)
    int16_t acc_y = 0;      // Y轴加速度 (LSB)
    int16_t acc_z = 0;      // Z轴加速度 (LSB)
    int16_t gyr_x = 0;      // X轴角速度 (LSB)
    int16_t gyr_y = 0;      // Y轴角速度 (LSB)
    int16_t gyr_z = 0;      // Z轴角速度 (LSB)
    float AngleX = 0.0;     // X轴倾角 (度)
    float AngleY = 0.0;     // Y轴倾角 (度)
    float AngleZ = 0.0;     // Z轴倾角 (度)
    int motion = 0;         // 运动等级 (0-3)
    
    std::string ToString() const;
} t_sQMI8658;
```

### motion_level_t
```cpp
typedef enum {
    MOTION_LEVEL_IDLE = 0,      // 静止状态
    MOTION_LEVEL_SLIGHT = 1,    // 轻微运动
    MOTION_LEVEL_MODERATE = 2,  // 中等运动
    MOTION_LEVEL_INTENSE = 3    // 剧烈运动
} motion_level_t;
```

## 🔧 寄存器定义

### 关键寄存器
```cpp
enum qmi8658_reg {
    QMI8658_WHO_AM_I = 0x00,    // 设备ID寄存器
    QMI8658_CTRL1 = 0x02,       // 控制寄存器1
    QMI8658_CTRL2 = 0x03,       // 加速度计配置
    QMI8658_CTRL3 = 0x04,       // 陀螺仪配置
    QMI8658_CTRL7 = 0x08,       // 使能寄存器
    QMI8658_STATUS0 = 0x2E,     // 状态寄存器
    QMI8658_AX_L = 0x35,        // 数据寄存器起始地址
    QMI8658_RESET = 0x60        // 复位寄存器
};
```

### 配置值
```cpp
// 初始化配置
CTRL1: 0x60  // 地址自动增加
CTRL7: 0x03  // 允许加速度和陀螺仪
CTRL2: 0x15  // ACC 4g 250Hz
CTRL3: 0x00  // GRY 512dps 250Hz
RESET: 0xb0  // 软件复位
```

## 📈 数据换算

### 换算常数
```cpp
const float ACC_LSB_TO_G = 1.0f / 8192.0f;      // 加速度LSB到g
const float GYR_LSB_TO_DPS = 1.0f / 64.0f;      // 陀螺仪LSB到度/秒
const float RAD_TO_DEG = 57.29578f;              // 弧度到度
```

### 使用示例
```cpp
// 换算加速度
float acc_x_g = data.acc_x * ACC_LSB_TO_G;

// 换算角速度
float gyr_x_dps = data.gyr_x * GYR_LSB_TO_DPS;

// 四舍五入到4位小数
float rounded = roundf(value * 10000.0f) / 10000.0f;
```

## 🎯 运动检测算法

### 算法参数
```cpp
// 定点数换算系数
static const int32_t LSB_TO_G_FIXED = 8;  // 1/8192 in Q16.16

// 运动阈值（定点数）
IDLE_THRESHOLD = 3277;      // 0.05g
SLIGHT_THRESHOLD = 13107;   // 0.2g  
MODERATE_THRESHOLD = 26214; // 0.4g
```

### 检测流程
```cpp
1. 转换当前加速度到定点数
2. 计算与历史值的差值
3. 累加三轴变化量
4. 与阈值比较确定运动等级
5. 更新历史值
```

## 🔍 错误处理

### 返回值检查
```cpp
// 初始化检查
if (!imu.Initialize()) {
    ESP_LOGE(TAG, "IMU init failed");
    return ESP_FAIL;
}

// 数据读取检查
if (!imu.ReadMotionData(&data)) {
    ESP_LOGW(TAG, "IMU read failed");
    return;
}
```

### 异常处理
```cpp
try {
    imu.ReadRegs(reg, data, len);
} catch (...) {
    ESP_LOGE(TAG, "I2C communication error");
    return false;
}
```

## 📊 性能指标

### 时序要求
- **初始化时间**: ~10ms
- **数据读取**: ~1ms
- **运动检测**: ~0.1ms
- **角度计算**: ~0.5ms

### 内存使用
- **类实例**: ~64字节
- **数据结构**: ~32字节
- **栈使用**: ~128字节

### 功耗特性
- **工作电流**: ~0.5mA
- **待机电流**: ~10μA
- **采样功耗**: 与频率成正比

## 🛠️ 集成指南

### 板级集成
```cpp
// 在板级配置中
class LichuangDevBoard : public WifiBoard {
private:
    QMI8658* imu_sensor_;
    
public:
    virtual QMI8658* GetIMUSensor() override {
        return imu_sensor_;
    }
};
```

### 应用层集成
```cpp
// 在Application中
void Application::InitializeIMU() {
    imu_sensor_ = board.GetIMUSensor();
    if (imu_sensor_ && imu_sensor_->IsInitialized()) {
        // 创建定时器
        esp_timer_start_periodic(timer, 100000); // 100ms
    }
}
```

### MQTT集成
```cpp
// 发送IMU数据
void MqttProtocol::SendImuStatesAndValue(const t_sQMI8658& data, int touch) {
    // 创建JSON对象
    // 添加换算后的物理单位值
    // 发布到MQTT主题
}
```

## 📝 最佳实践

### 1. 初始化
- 在系统启动早期初始化IMU
- 检查初始化返回值
- 添加重试机制

### 2. 数据读取
- 使用定时器定期读取
- 检查数据有效性
- 处理读取失败情况

### 3. 运动检测
- 根据应用调整阈值
- 考虑环境噪声影响
- 实现防抖机制

### 4. 性能优化
- 避免频繁的浮点运算
- 使用定点数计算
- 合理设置采样频率
