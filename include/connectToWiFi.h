#ifndef _CONNECT_TO_WIFI_H_
#define _CONNECT_TO_WIFI_H_

#include <Arduino.h>
#include <WiFi.h>

// 当前项目直接写死 WiFi 参数，便于实验室联调。
#define WIFI_SSID "huage"
#define WIFI_PASSWORD "11111111"

// 尽早发起 STA 连接并记录起点时间，供后续按总预算等待（可与 HX711 等初始化重叠）。
// 多次调用安全：仅第一次生效。
void startWiFiConnect();

// 是否已调用过 startWiFiConnect()。
bool wifiBootConnectStarted();

// 首次 startWiFiConnect() 时的 millis()，未启动则返回 0。
uint32_t wifiBootConnectStartMillis();

// 兼容旧名：等价于 startWiFiConnect()。
void setupWiFi();

#endif
