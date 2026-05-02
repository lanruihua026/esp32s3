#ifndef APP_STATE_H
#define APP_STATE_H

#include <Arduino.h>
#include <Preferences.h>

// 硬件引脚与系统节流参数集中在此处，便于答辩时对照总体架构说明。
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

// HX711 标定与保护阈值。空仓阈值用于避免带载上电时误写零点。
static constexpr float HX711_CAL_FACTOR_1 = 398.0f;
static constexpr float HX711_CAL_FACTOR_2 = 415.9f;
static constexpr float HX711_CAL_FACTOR_3 = 421.8f;
static constexpr bool HX711_WRITE_DEFAULT_CAL_TO_NVS = true;
static constexpr uint32_t HX711_CAL_PROFILE_VERSION = 20260502;
static constexpr float HX711_BOOT_EMPTY_THRESHOLD_G = 50.0f;

// OLED 与称重保护参数。
static constexpr uint32_t OLED_MIN_UPDATE_INTERVAL_MS = 50;
static constexpr int32_t HX711_MAX_RANGE_G = 5000;

// 可由云端配置或 NVS 恢复的运行参数。
extern int32_t g_fullWeightG;
extern float g_aiConfThreshold;

// NVS 配置句柄。
extern Preferences gPrefs;
extern bool g_prefsOk;

// ESP32-CAM 串口接收缓存。
extern HardwareSerial CameraUart;
extern char g_camLineBuf[128];
extern size_t g_camLinePos;

// 最近一次 AI 链路状态；由 cameraAi.cpp 更新，OLED 与舵机逻辑读取。
extern bool g_lastAiDetected;
extern char g_lastAiLabel[32];
extern float g_lastAiConf;
extern uint32_t g_lastAiUpdateMs;
extern bool g_camReady;
extern bool g_aiOfflineReported;
extern uint32_t g_lastCamHeartbeatMs;
extern uint32_t g_lastAiSuccessMs;

// 三路称重实时值。
extern int32_t g_currentWeight1;
extern int32_t g_currentWeight2;
extern int32_t g_currentWeight3;

// 主循环节流时间戳。
extern uint32_t g_lastReportMs;
extern uint32_t g_lastSampleMs;
extern uint32_t g_lastWiFiRetryMs;
extern uint32_t g_lastOledUpdateMs;

// 启动与报警状态。
extern bool g_hx711_1InitOk;
extern bool g_hx711_2InitOk;
extern bool g_hx711_3InitOk;
extern bool g_wifiInitTimeout;
extern bool g_warningActive;

#endif
