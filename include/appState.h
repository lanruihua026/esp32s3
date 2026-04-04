#ifndef APP_STATE_H
#define APP_STATE_H

#include <Arduino.h>
#include <Preferences.h>

static constexpr uint8_t WARNING_LIGHT_PIN = 6;               // 警示灯
static constexpr uint8_t RGB_LED_PIN = 38;                    // 板载rgb
static constexpr uint8_t RGB_LED_COUNT = 1;                   // 板载rgb一颗
static constexpr uint8_t CAM_UART_RX_PIN = 18;                // 板载串口接收来自esp32cam的模型识别结果
static constexpr uint8_t CAM_UART_TX_PIN = 17;                // 暂时用不到
static constexpr uint32_t CAM_UART_BAUD = 115200;             // 板载串口波特率
static constexpr int32_t WARNING_RELEASE_HYSTERESIS_G = 100;  // 超重后恢复正常的重量差阈值，单位克。例如超重1000g，只有当重量降到900g以下才认为恢复正常，避免因重量波动导致警示灯频闪
static constexpr uint32_t WEIGHT_SAMPLE_INTERVAL_MS = 250;    // 称重采集间隔
static constexpr uint32_t PROPERTY_REPORT_INTERVAL_MS = 3000; // 属性上报间隔
static constexpr uint32_t WIFI_RETRY_INTERVAL_MS = 15000;     // WiFi重试间隔
static constexpr uint32_t WIFI_BOOT_WAIT_MS = 5000;           // WiFi启动等待时间
static constexpr uint32_t CAM_HEARTBEAT_TIMEOUT_MS = 3000;    // 相机心跳超时时间
static constexpr uint32_t AI_OFFLINE_TIMEOUT_MS = 5000;       // AI离线超时时间
static constexpr float HX711_CAL_FACTOR_1 = 452.3f;           // HX711校准系数
static constexpr float HX711_CAL_FACTOR_2 = 454.5f;           // HX711校准系数
static constexpr float HX711_CAL_FACTOR_3 = 415.6f;           // HX711校准系数
static constexpr float HX711_BOOT_EMPTY_THRESHOLD_G = 50.0f;  // HX711启动时的空载重量阈值，单位克。启动时如果称重读数超过这个值，认为秤盘上有物品，提示用户清空秤盘后重启设备
static constexpr uint32_t OLED_MIN_UPDATE_INTERVAL_MS = 50;   // OLED最小更新间隔，单位毫秒。OLED刷新较慢，过快的更新可能导致显示异常或闪烁，这个参数可以限制OLED的刷新频率

extern int32_t g_fullWeightG;   // 全局变量，满载重量阈值，单位克。这个值可以通过NVS（非易失性存储）配置，如果NVS不可用则使用默认值1000g
extern float g_aiConfThreshold; // 全局变量，AI识别置信度阈值。这个值可以通过NVS配置，如果NVS不可用则使用默认值0.0f，表示不启用置信度过滤

extern Preferences gPrefs; // 全局Preferences对象，用于访问NVS存储的配置项
extern bool g_prefsOk;     // 全局变量，表示Preferences是否成功初始化。如果为false，表示无法访问NVS，应用将使用默认配置并且配置更改不会被保存到NVS。

extern HardwareSerial CameraUart; // 全局变量，用于与相机通信的串口对象
extern char g_camLineBuf[128];    // 全局变量，用于存储相机发送的行数据
extern size_t g_camLinePos;       // 全局变量，表示当前行数据的位置

extern bool g_lastAiDetected;         // 全局变量，表示上次是否检测到物体
extern char g_lastAiLabel[32];        // 全局变量，存储上次检测到的物体标签
extern float g_lastAiConf;            // 全局变量，存储上次检测到的物体的置信度
extern uint32_t g_lastAiUpdateMs;     // 全局变量，存储上次AI结果更新的时间戳（毫秒）。这个变量用于判断AI结果是否过时，例如如果当前时间与这个时间戳的差值超过AI_OFFLINE_TIMEOUT_MS，就认为AI离线了。
extern bool g_camReady;               // 全局变量，表示相机是否准备好。这个变量可以在相机发送特定的心跳消息时设置为true，如果超过CAM_HEARTBEAT_TIMEOUT_MS没有收到心跳消息，就设置为false，表示相机可能离线了。
extern bool g_aiOfflineReported;      // 全局变量，表示是否已经报告过AI离线状态。这个变量用于避免重复报告AI离线的警告，例如当检测到AI离线时，如果这个变量为false，就触发警告并将其设置为true；当AI恢复在线时，将其重置为false，以便下次如果再次离线时能够再次报告。
extern uint32_t g_lastCamHeartbeatMs; // 全局变量，存储上次收到相机心跳消息的时间戳（毫秒）。这个变量用于判断相机是否在线，例如如果当前时间与这个时间戳的差值超过CAM_HEARTBEAT_TIMEOUT_MS，就认为相机离线了。
extern uint32_t g_lastAiSuccessMs;    // 全局变量，存储上次成功检测到物体的时间戳（毫秒）。这个变量可以用于判断AI是否长时间没有检测到物体，例如如果当前时间与这个时间戳的差值超过AI_OFFLINE_TIMEOUT_MS，就认为AI可能有问题了。

extern int32_t g_currentWeight1; // 全局变量，1号HX711当前重量读数
extern int32_t g_currentWeight2; // 全局变量，2号HX711当前重量读数
extern int32_t g_currentWeight3; // 全局变量，3号HX711当前重量读数

extern uint32_t g_lastReportMs;     // 全局变量，存储上次属性上报的时间戳（毫秒）。这个变量用于控制属性上报的频率，例如如果当前时间与这个时间戳的差值超过PROPERTY_REPORT_INTERVAL_MS，就触发一次属性上报并更新这个时间戳。
extern uint32_t g_lastSampleMs;     // 全局变量，存储上次称重采样的时间戳（毫秒）。这个变量用于控制称重采样的频率，例如如果当前时间与这个时间戳的差值超过WEIGHT_SAMPLE_INTERVAL_MS，就触发一次称重采样并更新这个时间戳。
extern uint32_t g_lastWiFiRetryMs;  // 全局变量，存储上次WiFi重试的时间戳（毫秒）。这个变量用于控制WiFi重试的频率，例如如果当前时间与这个时间戳的差值超过WIFI_RETRY_INTERVAL_MS，就触发一次WiFi重试并更新这个时间戳。
extern uint32_t g_lastOledUpdateMs; // 全局变量，存储上次OLED更新的时间戳（毫秒）。这个变量用于控制OLED更新的频率，例如如果当前时间与这个时间戳的差值超过OLED_MIN_UPDATE_INTERVAL_MS，就触发一次OLED更新并更新这个时间戳。

extern bool g_hx711_1InitOk;   // 全局标志位，表示1号HX711是否成功初始化
extern bool g_hx711_2InitOk;   // 全局标志位，表示2号HX711是否成功初始化
extern bool g_hx711_3InitOk;   // 全局标志位，表示3号HX711是否成功初始化
extern bool g_wifiInitTimeout; // 全局标志位，表示WiFi初始化是否超时。如果在启动过程中等待WiFi连接的时间超过WIFI_BOOT_WAIT_MS，就将这个标志设置为true，表示WiFi启动失败，应用可以根据这个标志来决定是否进入离线模式或者提示用户检查网络设置。
extern bool g_warningActive;   // 声光报警闩锁：未按「解除警告」时，任一路≥满载则置 true；用户按键解除后会临时为 false，此时由 binControl 内静音锁抑制声光；三路均低于「满载−滞回」时清 false。不直接驱动 OLED（屏上超重仍按重量与阈值计算）。

#endif
