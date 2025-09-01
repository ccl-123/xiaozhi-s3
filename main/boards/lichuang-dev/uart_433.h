#ifndef UART_433_H
#define UART_433_H

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "config.h"

// 433MHz UART配置参数从config.h获取
// #define UART_433_PIN_NUM           UART_NUM_2    // 使用UART2
// #define UART_433_TX_PIN            GPIO_NUM_6    // 发送引脚
// #define UART_433_RX_PIN            GPIO_NUM_7    // 接收引脚
// #define UART_433_TX_Buffer_Size    (1024)       // 发送缓冲区
// #define UART_433_RX_Buffer_Size    (1024)       // 接收缓冲区

#ifdef __cplusplus
extern "C" {
#endif

// 全局变量声明
extern char button_value;        // 按键字符值
extern int button_value_int;     // 按键数值
extern bool key_433_press;       // 按键状态标志
extern int Key_433_Count;        // 接收计数器
extern int Key_433_Count_Now;    // 当前计数器

/**
 * @brief 初始化433MHz UART通信
 */
void UART_433_Init(void);

/**
 * @brief 接收433MHz数据并解析
 */
void UART_433_RX_DATA(void);

/**
 * @brief 发送数据到433MHz模块
 * @param data 要发送的数据字符串
 */
void UART_433_TX_DATA(const char* data);

/**
 * @brief 检查433MHz UART是否已初始化
 * @return true 如果已初始化，false 如果未初始化
 */
bool UART_433_IsInitialized(void);

#ifdef __cplusplus
}
#endif

#endif // UART_433_H
