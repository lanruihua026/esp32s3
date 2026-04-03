#ifndef APP_STATE_H
#define APP_STATE_H

#include <Arduino.h>
#include <Preferences.h>

static constexpr uint8_t WARNING_LIGHT_PIN = 6;
static constexpr uint8_t RGB_LED_PIN = 38;
static constexpr uint8_t RGB_LED_COUNT = 1;
static constexpr uint8_t CAM_UART_RX_PIN = 18;
static constexpr uint8_t CAM_UART_TX_PIN = 17;
static constexpr uint32_t CAM_UART_BAUD = 115200;
static constexpr int32_t WARNING_RELEASE_HYSTERESIS_G = 100;
static constexpr uint32_t WEIGHT_SAMPLE_INTERVAL_MS = 250;
static constexpr uint32_t PROPERTY_REPORT_INTERVAL_MS = 3000;
static constexpr uint32_t WIFI_RETRY_INTERVAL_MS = 15000;
static constexpr uint32_t WIFI_BOOT_WAIT_MS = 5000;
static constexpr uint32_t CAM_HEARTBEAT_TIMEOUT_MS = 3000;
static constexpr uint32_t AI_OFFLINE_TIMEOUT_MS = 5000;
static constexpr float HX711_CAL_FACTOR_1 = 452.3f;
static constexpr float HX711_CAL_FACTOR_2 = 454.5f;
static constexpr float HX711_CAL_FACTOR_3 = 415.6f;
static constexpr float HX711_BOOT_EMPTY_THRESHOLD_G = 50.0f;
static constexpr uint32_t OLED_MIN_UPDATE_INTERVAL_MS = 50;

extern int32_t g_fullWeightG;
extern float g_aiConfThreshold;

extern Preferences gPrefs;
extern bool g_prefsOk;

extern HardwareSerial CameraUart;
extern char g_camLineBuf[128];
extern size_t g_camLinePos;

extern bool g_lastAiDetected;
extern char g_lastAiLabel[32];
extern float g_lastAiConf;
extern uint32_t g_lastAiUpdateMs;
extern bool g_camReady;
extern bool g_aiOfflineReported;
extern uint32_t g_lastCamHeartbeatMs;
extern uint32_t g_lastAiSuccessMs;

extern int32_t g_currentWeight1;
extern int32_t g_currentWeight2;
extern int32_t g_currentWeight3;

extern uint32_t g_lastReportMs;
extern uint32_t g_lastSampleMs;
extern uint32_t g_lastWiFiRetryMs;
extern uint32_t g_lastOledUpdateMs;

extern bool g_hx711_1InitOk;
extern bool g_hx711_2InitOk;
extern bool g_hx711_3InitOk;
extern bool g_wifiInitTimeout;
extern bool g_warningActive;

#endif
