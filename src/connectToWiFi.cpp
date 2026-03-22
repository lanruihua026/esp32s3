#include "connectToWiFi.h"

void setupWiFi()
{
    // 切到 STA 模式，让开发板以“终端设备”身份接入路由器。
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    // 这里只发起连接，是否连接成功由 main.cpp 的启动流程和后续重连逻辑负责。
    Serial.println("WiFi connect requested");
}
