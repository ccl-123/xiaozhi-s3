#include "uart_433.h"

static const char* TAG = "UART_433";

// 全局变量定义
char button_value = '0';         // 按键字符值
int button_value_int = 0;        // 按键数值
bool key_433_press = false;      // 按键状态标志
int Key_433_Count = 0;           // 接收计数器
int Key_433_Count_Now = 0;       // 当前计数器

// 433MHz UART DMA配置选项
#define USE_DMA_FOR_433_UART      1    // 1=启用DMA, 0=标准模式

void UART_433_Init(void) {
    ESP_LOGI(TAG, "Initializing 433MHz UART...");

    // 首先检查UART是否已经安装，如果是则先卸载
    if (uart_is_driver_installed(UART_433_PIN_NUM)) {
        ESP_LOGW(TAG, "UART driver already installed, uninstalling first...");
        uart_driver_delete(UART_433_PIN_NUM);
    }

    uart_config_t uart_config = {};
    uart_config.baud_rate = 9600;                   // 波特率9600
    uart_config.data_bits = UART_DATA_8_BITS;       // 8位数据位
    uart_config.parity = UART_PARITY_DISABLE;       // 无奇偶校验
    uart_config.stop_bits = UART_STOP_BITS_1;       // 1个停止位
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE; // 无流控
    uart_config.source_clk = UART_SCLK_DEFAULT;     // 默认时钟源
    uart_config.rx_flow_ctrl_thresh = 0;            // 流控阈值
    
    // 配置UART参数
    esp_err_t ret = uart_param_config(UART_433_PIN_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART parameters: %s", esp_err_to_name(ret));
        return;
    }

    // 设置UART引脚
    ret = uart_set_pin(UART_433_PIN_NUM, UART_433_TX_PIN, UART_433_RX_PIN, -1, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        return;
    }

#if USE_DMA_FOR_433_UART == 1
    // DMA模式：高性能，支持大数据量传输
    ESP_LOGI(TAG, "Installing UART driver with DMA support...");
    ret = uart_driver_install(UART_433_PIN_NUM,
                             UART_433_RX_Buffer_Size,     // RX缓冲区
                             UART_433_TX_Buffer_Size,     // TX缓冲区
                             10,                          // 事件队列大小
                             NULL,                        // 事件队列句柄
                             ESP_INTR_FLAG_IRAM);         // DMA中断标志
#else
    // 标准模式：兼容旧项目配置
    ESP_LOGI(TAG, "Installing UART driver in standard mode...");
    ret = uart_driver_install(UART_433_PIN_NUM,
                             UART_433_RX_Buffer_Size,     // RX缓冲区
                             UART_433_TX_Buffer_Size,     // TX缓冲区（必须>0以支持写操作）
                             0,                           // 无事件队列
                             NULL,                        // 无事件队列句柄
                             0);                          // 标准中断标志
#endif

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "433MHz UART initialized successfully on GPIO%d(TX)/GPIO%d(RX) with %s mode",
             UART_433_TX_PIN, UART_433_RX_PIN,
             USE_DMA_FOR_433_UART ? "DMA" : "Standard");
}

void UART_433_RX_DATA(void) {
    size_t len = 0;
    uint8_t uart_433_data[UART_433_RX_Buffer_Size] = {0};
    
    // 获取缓冲区数据长度
    uart_get_buffered_data_len(UART_433_PIN_NUM, &len); 
    
    if(len > 0) {
        Key_433_Count += 1;  // 接收计数器
        memset(uart_433_data, 0, UART_433_RX_Buffer_Size); 
        
        // 读取数据，超时时间100ms
        int bytes_read = uart_read_bytes(UART_433_PIN_NUM, uart_433_data, len, 100 / portTICK_PERIOD_MS);
        
        if (bytes_read > 0) {
            ESP_LOGD(TAG, "Received %d bytes: %.*s", bytes_read, bytes_read, uart_433_data);
            
            // 协议解析：查找固定帧头"LC:"
            const char* frame_header = "LC:";  // 修复：使用const char*避免警告
            char* frame_data = (char*)uart_433_data;
            char* header_position = strstr(frame_data, frame_header);
            
            if (header_position != NULL) {  
                // 提取按键值部分（跳过帧头+5个字符）
                char* button_value_str = header_position + strlen(frame_header) + 5;  
                
                if (button_value_str < (char*)uart_433_data + bytes_read) {
                    button_value = *button_value_str;  
                    
                    // 十六进制字符转换为数值
                    if (button_value >= '0' && button_value <= '9') {
                        button_value_int = button_value - '0';  // '0'~'9' → 0~9
                    } else if (button_value >= 'A' && button_value <= 'F') {
                        button_value_int = button_value - 'A' + 10;  // 'A'~'F' → 10~15
                    } else if (button_value >= 'a' && button_value <= 'f') {
                        button_value_int = button_value - 'a' + 10;  // 'a'~'f' → 10~15
                    }
                    
                    ESP_LOGI(TAG, "Button pressed: '%c' (value: %d)", button_value, button_value_int);
                }
            }
        }
        
        // 按键状态检测
        if(Key_433_Count_Now != Key_433_Count) {
            key_433_press = true;   // 检测到新按键
            Key_433_Count_Now = Key_433_Count;
            ESP_LOGI(TAG, "New key press detected, count: %d", Key_433_Count);
        } else {
            key_433_press = false;
        }
    }
}

void UART_433_TX_DATA(const char* data) {
    if (data == nullptr) {
        ESP_LOGW(TAG, "TX data is null");
        return;
    }

    ESP_LOGD(TAG, "Sending data: %s", data);

    // 发送数据并检查返回值
    int bytes_written = uart_write_bytes(UART_433_PIN_NUM, data, strlen(data));
    if (bytes_written < 0) {
        ESP_LOGE(TAG, "Failed to write data: %d", bytes_written);
        return;
    }

    // 发送换行符
    bytes_written = uart_write_bytes(UART_433_PIN_NUM, "\r\n", 2);
    if (bytes_written < 0) {
        ESP_LOGE(TAG, "Failed to write newline: %d", bytes_written);
        return;
    }

    ESP_LOGD(TAG, "Successfully sent %zu bytes + newline", strlen(data));
}

bool UART_433_IsInitialized(void) {
    return uart_is_driver_installed(UART_433_PIN_NUM);
}
