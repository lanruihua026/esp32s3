#ifndef _CONNECT_TO_WIFI_H_
#define _CONNECT_TO_WIFI_H_

#include <Arduino.h>
#include <WiFi.h>

// WiFi 凭据从 secrets.h 读取，该文件不进入版本库。
// 首次部署请复制 secrets.h.example → secrets.h 并填入真实值。
#include "secrets.h"

// 尽早发起 STA 连接并记录起点时间，供后续按总预算等待（可与 HX711 等初始化重叠）。
// 多次调用安全：仅第一次生效。
void startWiFiConnect();

// 是否已调用过 startWiFiConnect()。
bool wifiBootConnectStarted();

// 首次 startWiFiConnect() 时的 millis()，未启动则返回 0。
uint32_t wifiBootConnectStartMillis();

#endif
