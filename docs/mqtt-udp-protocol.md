# MQTT+UDP 协议交互文档

本文档基于 xiaozhi-afe 项目源码（application.cc、mqtt_protocol.cc、protocol.cc）整理，供后端开发人员参考实现。

## 概述

xiaozhi-afe 设备使用 MQTT+UDP 混合协议进行通信：
- **MQTT 控制通道**：传输 JSON 格式的控制消息（TLS 8883）
- **UDP 音频通道**：传输 AES-CTR 加密的 OPUS 音频数据

## 1. 配置约定

设备通过 OTA 接口获取 MQTT 配置，存储在 Settings("mqtt") 命名空间：

```json
{
  "mqtt": {
    "endpoint": "mqtt.example.com:8883",
    "client_id": "device_12345",
    "username": "device_user",
    "password": "device_pass",
    "keepalive": 240,
    "publish_topic": "device/12345/control"
  }
}
```

## 2. 握手流程

### 2.1 连接建立

1. 设备连接 MQTT Broker（TLS 8883）
2. 设备发送 Hello 消息到 `publish_topic`
3. 服务端回复 ServerHello 到设备订阅的主题
4. 设备建立 UDP 连接并初始化 AES-CTR 加密

### 2.2 Hello 消息（设备→服务端）

```json
{
  "type": "hello",
  "version": 3,
  "transport": "udp",
  "features": {
    "aec": true,
    "mcp": true
  },
  "audio_params": {
    "format": "opus",
    "sample_rate": 16000,
    "channels": 1,
    "frame_duration": 60
  }
}
```

### 2.3 ServerHello 消息（服务端→设备）

```json
{
  "type": "hello",
  "transport": "udp",
  "session_id": "xxxx-uuid-xxxx",
  "audio_params": {
    "sample_rate": 16000,
    "frame_duration": 60
  },
  "udp": {
    "server": "1.2.3.4",
    "port": 20000,
    "key": "00112233445566778899AABBCCDDEEFF",
    "nonce": "0100000000000000000000000000000000000000"
  }
}
```

**字段说明：**
- `session_id`: 会话标识符，后续消息需携带
- `udp.key`: 128位 AES 密钥（32字符 HEX）
- `udp.nonce`: 16字节基准 nonce（32字符 HEX）

## 3. MQTT 控制消息

### 3.1 设备→服务端消息

#### 监听控制
```json
// 开始监听
{
  "session_id": "xxxx-uuid-xxxx",
  "type": "listen",
  "state": "start",
  "mode": "realtime|auto|manual"
}

// 停止监听
{
  "session_id": "xxxx-uuid-xxxx",
  "type": "listen",
  "state": "stop"
}

// 唤醒词检测
{
  "session_id": "xxxx-uuid-xxxx",
  "type": "listen",
  "state": "detect",
  "text": "小智小智"
}
```

#### 中止说话
```json
{
  "session_id": "xxxx-uuid-xxxx",
  "type": "abort",
  "reason": "wake_word_detected"
}
```

#### MCP 消息
```json
{
  "session_id": "xxxx-uuid-xxxx",
  "type": "mcp",
  "payload": {
    "tool": "set_led",
    "args": {"color": "#00FF00"}
  }
}
```

### 3.2 服务端→设备消息

#### TTS 控制
```json
// TTS 开始
{"type": "tts", "state": "start"}

// TTS 分句（用于 UI 显示）
{"type": "tts", "state": "sentence_start", "text": "你好，我在！"}

// TTS 结束
{"type": "tts", "state": "stop"}
```

#### STT 结果
```json
{"type": "stt", "text": "现在几点了？"}
```

#### LLM 情绪
```json
{"type": "llm", "emotion": "happy"}
```

#### MCP 响应
```json
{
  "type": "mcp",
  "payload": {
    "result": "success",
    "data": {...}
  }
}
```

#### 系统控制
```json
{"type": "system", "command": "reboot"}
```

#### 告警消息
```json
{
  "type": "alert",
  "status": "ERROR",
  "message": "服务器不可用",
  "emotion": "sad"
}
```

#### 会话关闭
```json
{"type": "goodbye", "session_id": "xxxx-uuid-xxxx"}
```

## 4. UDP 音频通道

### 4.1 数据包格式

```
UDP 包结构（网络字节序）：
+--------+--------+--------+--------+--------+--------+--------+--------+
| type   | flags  |    payload_len    |           ssrc            |
| (1B)   | (1B)   |       (2B)        |           (4B)            |
+--------+--------+--------+--------+--------+--------+--------+--------+
|                    timestamp (4B)                    |
+--------+--------+--------+--------+--------+--------+--------+--------+
|                    sequence (4B)                     |
+--------+--------+--------+--------+--------+--------+--------+--------+
|                                                               |
|                    Encrypted OPUS Payload                    |
|                        (payload_len bytes)                   |
+--------+--------+--------+--------+--------+--------+--------+
```

**字段说明：**
- `type`: 固定为 0x01
- `flags`: 保留字段
- `payload_len`: OPUS 数据长度
- `ssrc`: 保留字段
- `timestamp`: 毫秒时间戳（用于服务端 AEC）
- `sequence`: 递增序列号（从 1 开始）

### 4.2 AES-CTR 加密

#### 加密流程
1. 复制 ServerHello 中的基准 nonce（16字节）
2. 在 nonce 中填入当前包的 payload_len、timestamp、sequence
3. 使用该 nonce 进行 AES-CTR 加密
4. 发送：16字节头 + 加密后的 OPUS 数据

#### Nonce 构造
```
基准 nonce: 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
填充后:     01 00 [len] 00 00 00 00 [timestamp] [sequence]
           ↑     ↑                  ↑           ↑
         type  payload_len      timestamp   sequence
         (0)     (2-3)           (8-11)     (12-15)
```

### 4.3 序列号管理

- **发送端**：local_sequence 从 1 开始递增
- **接收端**：校验 remote_sequence 严格递增
  - 如果 `sequence < remote_sequence`：丢弃（旧包）
  - 如果 `sequence != remote_sequence + 1`：警告（丢包）
  - 更新 `remote_sequence = sequence`

## 5. 错误处理与重连

### 5.1 超时机制
- **通道超时**：120秒未收到任何数据（MQTT 或 UDP）
- **连接超时**：握手阶段 10秒超时

### 5.2 错误类型
- `SERVER_NOT_FOUND`: MQTT 配置缺失
- `SERVER_NOT_CONNECTED`: MQTT 连接失败
- `SERVER_TIMEOUT`: 握手超时
- `SERVER_ERROR`: 发送失败

### 5.3 重连策略
- MQTT 断开：下次 OpenAudioChannel 时重连
- UDP 错误：通过 MQTT 发送 goodbye 关闭会话
- 设备收到错误后回到 Idle 状态

## 6. 安全机制

### 6.1 传输安全
- **MQTT**: TLS 1.2+ 加密（端口 8883）
- **UDP**: AES-128-CTR 加密

### 6.2 密钥管理
- AES 密钥通过 MQTT/TLS 安全分发
- 每个会话使用独立的密钥和 nonce
- 设备端严格校验包类型和序列号

### 6.3 防重放攻击
- 序列号严格递增校验
- 时间戳辅助验证（可选）
- 会话 ID 绑定

## 7. 实现建议

### 7.1 服务端架构
```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   MQTT Broker   │    │  Control Server │    │   UDP Gateway   │
│    (8883/TLS)   │◄──►│   (JSON处理)    │◄──►│  (音频处理)     │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

### 7.2 关键接口
- **MQTT 消息路由**：根据 client_id 路由到对应设备
- **UDP 音频处理**：实时 OPUS 编解码和 AES-CTR 加解密
- **会话管理**：维护 session_id 与设备的映射关系

### 7.3 性能优化
- UDP 音频包优先级高于 MQTT 控制消息
- 使用连接池管理 MQTT 连接
- 音频处理使用异步 I/O

## 8. 调试与监控

### 8.1 日志格式
```
[MQTT] Device device_12345 connected
[UDP] Audio packet received: seq=123, len=64, ts=1234567890
[ERROR] Invalid sequence number: expected=124, got=126
```

### 8.2 监控指标
- MQTT 连接数和消息吞吐量
- UDP 包丢失率和延迟
- 音频编解码错误率
- 会话建立成功率

---

## 附录：代码示例

### A.1 UDP 包解析（Python 示例）
```python
import struct
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes

def parse_udp_packet(data, aes_key):
    # 解析头部
    header = data[:16]
    type_val, flags, payload_len, ssrc, timestamp, sequence = struct.unpack('!BBHIII', header)
    
    if type_val != 0x01:
        raise ValueError(f"Invalid packet type: {type_val}")
    
    # AES-CTR 解密
    cipher = Cipher(algorithms.AES(aes_key), modes.CTR(header))
    decryptor = cipher.decryptor()
    opus_data = decryptor.update(data[16:]) + decryptor.finalize()
    
    return {
        'sequence': sequence,
        'timestamp': timestamp,
        'opus_data': opus_data
    }
```

### A.2 MQTT 消息发送（Node.js 示例）
```javascript
const mqtt = require('mqtt');

class XiaozhiMqttServer {
    constructor(brokerUrl) {
        this.client = mqtt.connect(brokerUrl);
        this.sessions = new Map();
    }
    
    sendTtsStart(deviceTopic, sessionId) {
        const message = {
            type: 'tts',
            state: 'start'
        };
        this.client.publish(deviceTopic, JSON.stringify(message));
    }
    
    sendServerHello(deviceTopic, sessionId, udpConfig) {
        const message = {
            type: 'hello',
            transport: 'udp',
            session_id: sessionId,
            audio_params: {
                sample_rate: 16000,
                frame_duration: 60
            },
            udp: udpConfig
        };
        this.client.publish(deviceTopic, JSON.stringify(message));
    }
}
```

---

