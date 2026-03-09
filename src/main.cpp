#include <Arduino.h>
#include "connectToWiFi.h"
#include "hx711.h"
#include "oledInit.h"
#include "onenetMqtt.h"

// ===== 业务参数 =====
#define Warnlight 6              // 告警灯 GPIO
#define FULL_WEIGHT 400          // 满载阈值
#define HX711_CAL_FACTOR 1000.0f // HX711 校准因子（raw/g）

// ===== ESP32-CAM -> ESP32-S3 串口参数 =====
// 接线说明：
// 1) ESP32-CAM TX -> ESP32-S3 RX（本例 RX=16）
// 2) ESP32-CAM RX <- ESP32-S3 TX（可选，本例 TX=15）
// 3) 两块板必须共地（GND 对 GND）
static const int CAM_UART_RX_PIN = 16;
static const int CAM_UART_TX_PIN = 15;
static const uint32_t CAM_UART_BAUD = 115200;

// 串口对象：用于接收 ESP32-CAM 输出的识别结果。
HardwareSerial CameraUart(1);

// 串口接收缓存：按“逐字符收、按行解析”策略处理。
static char g_camLineBuf[128] = {0};
static size_t g_camLinePos = 0;

// 最近一次 AI 识别结果缓存（当前先用于日志和后续扩展）。
static bool g_lastAiDetected = false;
static char g_lastAiLabel[32] = "none";
static float g_lastAiConf = 0.0f;
static uint32_t g_lastAiUpdateMs = 0;

// ===== OneNET 设备身份 =====
// 说明：这些信息需要与你在 OneNET 平台创建的产品/设备保持一致。
static const char *ONENET_PRODUCT_ID = "f45hkc7xC7";
static const char *ONENET_DEVICE_NAME = "Box1";
static const char *ONENET_BASE64_KEY = "T0R5ejYyM1JrT2VuczBkZllINmZuazRicEMxc29xcnk=";

// 定时任务时间戳
static uint32_t lastReportMs = 0; // 上次上报属性时间
static uint32_t lastSampleMs = 0; // 上次采样重量时间

// 当前重量缓存（供 OLED 显示、告警判断、云上报复用）
static int32_t currentWeight = 0;

/**
 * @brief 解析一行来自 ESP32-CAM 的协议消息。
 *
 * 协议约定：
 * - DET,<label>,<conf>   识别成功
 * - NONE                本帧无目标
 * - ERR,<msg>           摄像头端或网络端异常
 *
 * 说明：
 * 该函数只负责解析和更新本地状态，不阻塞。
 */
void parseCameraLine(const char *line)
{
  if (line == nullptr || line[0] == '\0')
  {
    return;
  }

  // 1) 识别成功消息：DET,label,conf
  if (strncmp(line, "DET,", 4) == 0)
  {
    const char *p1 = strchr(line, ',');
    if (!p1)
    {
      return;
    }
    const char *p2 = strchr(p1 + 1, ',');
    if (!p2)
    {
      return;
    }

    // 提取 label（第二段）
    size_t labelLen = static_cast<size_t>(p2 - (p1 + 1));
    if (labelLen >= sizeof(g_lastAiLabel))
    {
      labelLen = sizeof(g_lastAiLabel) - 1;
    }
    memcpy(g_lastAiLabel, p1 + 1, labelLen);
    g_lastAiLabel[labelLen] = '\0';

    // 提取 conf（第三段）
    g_lastAiConf = atof(p2 + 1);
    g_lastAiDetected = true;
    g_lastAiUpdateMs = millis();

    Serial.printf("[CAM-UART] DET label=%s conf=%.3f\n", g_lastAiLabel, g_lastAiConf);
    return;
  }

  // 2) 无目标消息
  if (strcmp(line, "NONE") == 0)
  {
    g_lastAiDetected = false;
    strncpy(g_lastAiLabel, "none", sizeof(g_lastAiLabel) - 1);
    g_lastAiLabel[sizeof(g_lastAiLabel) - 1] = '\0';
    g_lastAiConf = 0.0f;
    g_lastAiUpdateMs = millis();

    Serial.println("[CAM-UART] NONE");
    return;
  }

  // 3) 异常消息
  if (strncmp(line, "ERR,", 4) == 0)
  {
    g_lastAiDetected = false;
    g_lastAiUpdateMs = millis();
    Serial.printf("[CAM-UART] %s\n", line);
    return;
  }

  // 4) 其他噪声行：打印出来便于调试，但不影响业务主流程。
  Serial.printf("[CAM-UART] Unknown line: %s\n", line);
}

/**
 * @brief 持续读取串口并按“换行符”切分消息。
 *
 * 实现细节：
 * 1. 逐字节读取，遇到 '\n' 触发一条完整消息解析。
 * 2. 忽略 '\r'，兼容 CRLF 与 LF。
 * 3. 超长行自动丢弃并复位，防止缓存溢出。
 */
void pollCameraUart()
{
  while (CameraUart.available())
  {
    char c = static_cast<char>(CameraUart.read());

    if (c == '\r')
    {
      continue;
    }

    if (c == '\n')
    {
      g_camLineBuf[g_camLinePos] = '\0';
      parseCameraLine(g_camLineBuf);
      g_camLinePos = 0;
      continue;
    }

    if (g_camLinePos < sizeof(g_camLineBuf) - 1)
    {
      g_camLineBuf[g_camLinePos++] = c;
    }
    else
    {
      // 若接收行过长，直接清空，避免内存越界。
      g_camLinePos = 0;
    }
  }
}

void setup()
{
  Serial.begin(115200);

  // 初始化来自 ESP32-CAM 的串口通道。
  CameraUart.begin(CAM_UART_BAUD, SERIAL_8N1, CAM_UART_RX_PIN, CAM_UART_TX_PIN);
  Serial.printf("[CAM-UART] Init done, baud=%u RX=%d TX=%d\n", CAM_UART_BAUD, CAM_UART_RX_PIN, CAM_UART_TX_PIN);

  // 1) OLED 初始化，并显示开机进度框架
  setupOLED();

  // 2) HX711 初始化与校准参数设置（20%~40%）
  showBootProgress(20, "HX711 Sensor");
  setupHX711();
  delay(100);

  showBootProgress(30, "Calibration");
  setCalibrationFactor(HX711_CAL_FACTOR);
  delay(100);

  // 3) WiFi 连接（40%~70%）
  showBootProgress(40, "WiFi Connecting");
  setupWiFi();

  // WiFi 连接通常需要数秒，这里用进度动画提升可视反馈
  uint8_t wifiProgress = 40;
  while (WiFi.status() != WL_CONNECTED && wifiProgress < 65)
  {
    delay(200);
    wifiProgress += 5;
    showBootProgress(wifiProgress, "WiFi Connecting");
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    showBootProgress(70, "WiFi Connected");
  }
  else
  {
    showBootProgress(70, "WiFi Failed");
  }
  delay(200);

  // 4) 告警灯 GPIO 初始化（70%~80%）
  showBootProgress(80, "Warning Light");
  pinMode(Warnlight, OUTPUT);
  delay(100);

  // 5) OneNET MQTT 初始化（80%~100%）
  // 注意：这里只做配置，真实连接在 loop() 内由 oneNetMqttLoop 自动完成。
  showBootProgress(85, "OneNET MQTT");
  OneNetMqttConfig cfg = {
      "mqtts.heclouds.com",
      1883,
      ONENET_PRODUCT_ID,
      ONENET_DEVICE_NAME,
      ONENET_BASE64_KEY,
      1893456000,
      OneNetSignMethod::SHA256};
  oneNetMqttBegin(cfg);

  showBootProgress(95, "System Ready");
  delay(200);
  showBootProgress(100, "Starting...");
  delay(300);

  // 初始化定时器
  lastReportMs = millis();
  lastSampleMs = millis();
  Serial.println("Setup complete");
}

void loop()
{
  uint32_t now = millis();

  // 先处理来自摄像头的识别串口数据，保证消息不会堆积。
  pollCameraUart();

  // 维护 MQTT 连接、收发和自动重连
  oneNetMqttLoop();

  // 持续刷新 OLED 综合状态页
  updateOLEDDisplay();

  // 每 500ms 采样一次重量
  if (now - lastSampleMs >= 500)
  {
    lastSampleMs = now;
    currentWeight = (int32_t)getWeight();
    setCurrentWeight(currentWeight);

    // 满载即点亮告警灯
    if (currentWeight >= FULL_WEIGHT)
    {
      digitalWrite(Warnlight, HIGH);
    }
    else
    {
      digitalWrite(Warnlight, LOW);
    }
  }

  // 每 10 秒上报一次属性（仅在 MQTT 已连接时执行）
  if (oneNetMqttConnected() && now - lastReportMs >= 10000)
  {
    lastReportMs = now;

    // 业务规则：达到阈值即判定满载
    bool isFull = (currentWeight >= FULL_WEIGHT);

    // OLED 上传状态置为 active
    setUploadingStatus(true);

    // 上传物模型属性：isFull + weight
    oneNetMqttUploadProperties(isFull, currentWeight);

    // 这里额外输出最近一次 AI 状态，方便你在串口联调时观察。
    if (g_lastAiDetected)
    {
      Serial.printf("[AI-STATE] label=%s conf=%.3f age=%lums\n", g_lastAiLabel, g_lastAiConf, (unsigned long)(now - g_lastAiUpdateMs));
    }
    else
    {
      Serial.printf("[AI-STATE] none age=%lums\n", (unsigned long)(now - g_lastAiUpdateMs));
    }
  }
}
