#ifndef SYSTEM_INIT_H
#define SYSTEM_INIT_H

#include <Arduino.h>

void syncRuntimeHealth();
void initBoardIndicators();
void initCameraUart();
void initOledModule();
void initHx711Modules();
void initWiFiWithTimeout();
void initServoModule();
void initMqttModule();
void initTimers();
void tryReconnectWiFi(uint32_t now);

#endif
