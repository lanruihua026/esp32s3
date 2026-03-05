#ifndef _CONNECT_TO_WIFI_H_
#define _CONNECT_TO_WIFI_H_

#include <Arduino.h>
#include <WiFi.h>

// WiFi 连接参数（当前为硬编码，实际项目建议改为配置文件或配网流程）
#define WIFI_SSID "huage"
#define WIFI_PASSWORD "11111111"

/**
 * @brief 连接到指定 WiFi 热点（STA 模式）
 *
 * 职责：
 * 1. 初始化 WiFi 工作模式为 STA。
 * 2. 发起连接。
 * 3. 在串口输出连接结果与基础网络信息。
 */
void setupWiFi();

#endif
