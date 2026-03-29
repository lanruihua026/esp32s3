#include "connectToWiFi.h"

namespace
{
    uint32_t s_wifiBootStartMs = 0;
    bool s_wifiBootStarted = false;
}

void startWiFiConnect()
{
    if (s_wifiBootStarted)
    {
        return;
    }
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    s_wifiBootStartMs = millis();
    s_wifiBootStarted = true;
}

bool wifiBootConnectStarted()
{
    return s_wifiBootStarted;
}

uint32_t wifiBootConnectStartMillis()
{
    return s_wifiBootStartMs;
}

void setupWiFi()
{
    startWiFiConnect();
}
