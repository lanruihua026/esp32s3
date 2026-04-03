#include "appState.h"

int32_t g_fullWeightG = 1000;
float g_aiConfThreshold = 0.0f;

Preferences gPrefs;
bool g_prefsOk = false;

HardwareSerial CameraUart(1);
char g_camLineBuf[128] = {0};
size_t g_camLinePos = 0;

bool g_lastAiDetected = false;
char g_lastAiLabel[32] = "none";
float g_lastAiConf = 0.0f;
uint32_t g_lastAiUpdateMs = 0;
bool g_camReady = false;
bool g_aiOfflineReported = false;
uint32_t g_lastCamHeartbeatMs = 0;
uint32_t g_lastAiSuccessMs = 0;

int32_t g_currentWeight1 = 0;
int32_t g_currentWeight2 = 0;
int32_t g_currentWeight3 = 0;

uint32_t g_lastReportMs = 0;
uint32_t g_lastSampleMs = 0;
uint32_t g_lastWiFiRetryMs = 0;
uint32_t g_lastOledUpdateMs = 0;

bool g_hx711_1InitOk = false;
bool g_hx711_2InitOk = false;
bool g_hx711_3InitOk = false;
bool g_wifiInitTimeout = false;
bool g_warningActive = false;
