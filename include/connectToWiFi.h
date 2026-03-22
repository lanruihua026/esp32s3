#ifndef _CONNECT_TO_WIFI_H_
#define _CONNECT_TO_WIFI_H_

#include <Arduino.h>
#include <WiFi.h>

// 当前项目直接写死 WiFi 参数，便于实验室联调。
#define WIFI_SSID "huage"
#define WIFI_PASSWORD "11111111"

// 发起 WiFi 连接。
// 本函数只负责开始连接，不负责长时间阻塞等待。
void setupWiFi();

#endif
