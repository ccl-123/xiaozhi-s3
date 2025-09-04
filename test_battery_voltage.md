# 电压检测功能测试指南

## 功能概述

本项目已成功集成电压检测和上报功能，参考了Aibox_37项目的实现。

## 实现的功能

### 1. 硬件层面
- **ADC初始化**：使用ADC_UNIT_1，配置ADC_CHANNEL_8和ADC_CHANNEL_9
- **电压采集**：GPIO8作为参考电压，GPIO9作为电池电压
- **计算公式**：`voltage = 4.98f * adc_raw_value / ref`

### 2. 应用层面
- **定期检测**：每500ms读取一次电压（125次IMU读取周期）
- **电压滤波**：`filtered_voltage = filtered_voltage * 0.85 + current_voltage * 0.15`
- **低电压检测**：当电压≤3.5V时触发上报
- **上报间隔**：30秒防重复上报机制

### 3. MQTT协议层面
- **电压转换**：将电压值转换为电池百分比字符串
- **JSON上报**：添加`battery_voltage`字段到IMU数据包
- **阈值定义**：
  - 3.5V → "25%"
  - 3.4V → "15%"  
  - 3.3V → "5%"

## 测试方法

### 1. 编译和烧录
```bash
cd xiaozhi-afe
idf.py build
idf.py flash monitor
```

### 2. 监控日志
查找以下关键日志：
```
I (xxx) LichuangDevBoard: ADC initialized for battery voltage detection (CH8=ref, CH9=battery)
W (xxx) Application: 🔋 Low battery detected: 3.45V (filtered: 3.44V)
I (xxx) MQTT: Sending IMU data: {"imu_type":0,"gx":0.0,"gy":0.0,"gz":0.0,"ax":0.0,"ay":0.0,"az":0.0,"touch_value":0,"battery_voltage":"25","device_id":"xxxxx","fall_state":0} to topic: doll/imu_status
```

### 3. MQTT消息验证
订阅MQTT主题`doll/imu_status`，检查消息中是否包含`battery_voltage`字段：
```json
{
  "imu_type": 0,
  "gx": 0.0,
  "gy": 0.0,
  "gz": 0.0,
  "ax": 0.0,
  "ay": 0.0,
  "az": 0.0,
  "touch_value": 0,
  "battery_voltage": "25",
  "device_id": "123456789",
  "fall_state": 0
}
```

## 电压阈值说明

| 电压范围 | 电池百分比 | 说明 |
|---------|-----------|------|
| > 3.5V  | ""        | 正常电压，不上报 |
| ≤ 3.5V  | "25"      | 低电压警告 |
| ≤ 3.4V  | "15"      | 电量不足 |
| ≤ 3.3V  | "5"       | 电量严重不足 |

## 触发条件

电压信息会在以下情况下上报：

1. **低电压定期上报**：电压≤3.5V时，每30秒上报一次
2. **跌倒检测事件**：检测到跌倒时，包含当前电压信息
3. **433MHz按键事件**：按键触发时，包含当前电压信息

## 故障排除

### 1. ADC读取失败
- 检查GPIO8和GPIO9的硬件连接
- 确认ADC配置正确

### 2. 电压值异常
- 检查参考电压是否稳定
- 验证计算公式是否适合硬件设计

### 3. MQTT上报失败
- 确认MQTT连接正常
- 检查设备状态是否允许上报（非Idle状态）

## 代码修改点

1. **mqtt_protocol.h/cc**：添加电压参数和转换函数
2. **board.h**：添加GetBattary()虚函数
3. **lichuang_dev_board.cc**：实现ADC初始化和电压读取
4. **application.h/cc**：添加电压检测逻辑和定时上报

## 与参考项目的差异

- 保持了相同的电压检测算法和阈值
- 集成到现有的IMU数据上报机制中
- 添加了防重复上报机制
- 优化了日志输出格式
