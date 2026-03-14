#include <Arduino.h>
#include "connectToWiFi.h"
#include "hx711.h"
#include "oledInit.h"
#include "onenetMqtt.h"
#include "buttonControl.h"
#include "servoControl.h"
// ===== 业务参数 =====
#define Warnlight 6             // 告警灯 GPIO
#define FULL_WEIGHT 1000        // 实际满载阈值（g）：HX711量程5000g，设定最大载重1000g
#define HX711_CAL_FACTOR 449.1f // HX711 校准因子（raw/g）：以216g砝码校准（原始值1000×97/216≈449.1）

// ===== ESP32-CAM -> ESP32-S3 串口参数 =====
// 接线说明：
// 1) ESP32-CAM TX -> ESP32-S3 RX（本例 RX=18）
// 2) ESP32-CAM RX <- ESP32-S3 TX（可选，本例 TX=17）
// 3) 两块板必须共地（GND 对 GND）
static const int CAM_UART_RX_PIN = 18;
static const int CAM_UART_TX_PIN = 17;
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

    // 同步更新 OLED 模块的 AI 识别结果缓存
    setAiResult(true, g_lastAiLabel, g_lastAiConf, g_lastAiUpdateMs);

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

    // 同步更新 OLED 缓存
    setAiResult(false, "none", 0.0f, g_lastAiUpdateMs);

    Serial.println("[CAM-UART] NONE");
    return;
  }

  // 3) 异常消息
  if (strncmp(line, "ERR,", 4) == 0)
  {
    g_lastAiDetected = false;
    g_lastAiUpdateMs = millis();

    // 异常时也更新 OLED 缓存
    setAiResult(false, "ERR", 0.0f, g_lastAiUpdateMs);

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

/**
 * @brief Arduino 初始化入口，系统上电后执行一次
 *
 * 初始化顺序：
 * 1. Serial / CameraUart 串口
 * 2. OLED 显示屏（含启动进度动画）
 * 3. HX711 传感器及校准因子设置
 * 4. WiFi 连接（带进度推进动画）
 * 5. 告警灯 GPIO
 * 6. 按键中断及页面切换回调
 * 7. OneNET MQTT 配置（实际连接延后到 loop()）
 */
void setup()
{
  Serial.begin(115200);

  // 初始化来自 ESP32-CAM 的串口通道。
  CameraUart.begin(CAM_UART_BAUD, SERIAL_8N1, CAM_UART_RX_PIN, CAM_UART_TX_PIN);
  // Serial.printf("[CAM-UART] Init done, baud=%u RX=%d TX=%d\n", CAM_UART_BAUD, CAM_UART_RX_PIN, CAM_UART_TX_PIN);

  // 1) OLED 初始化，并显示开机进度框架
  setupOLED();
  setFullWeight(FULL_WEIGHT); // 将满载阈值传入 OLED 模块

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

  initServo();      // 初始化舵机
  setServoAngle(0); // 默认角度
  showBootProgress(50, "Servo Initialized");

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
  showBootProgress(78, "Warning Light");
  pinMode(Warnlight, OUTPUT);
  delay(100);

  // 5) 按键初始化，并注册回调
  showBootProgress(86, "Buttons");
  setupButtons();
  setButton1Callback([]()
                     { setOledPage(1); }); // 按键1：切换到识别结果展示页
  setButton2Callback([]()
                     { setOledPage(0); }); // 按键2：切换到综合信息页
  delay(100);

  // 7) OneNET MQTT 初始化（86%~100%）
  // 注意：这里只做配置，真实连接在 loop() 内由 oneNetMqttLoop 自动完成。
  showBootProgress(90, "OneNET MQTT");
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

/**
 * @brief Arduino 主循环，持续执行
 *
 * 每帧执行顺序：
 * 1. pollCameraUart()   — 接收并解析 ESP32-CAM 串口数据
 * 2. pollButtons()      — 消抖处理按键并触发回调
 * 3. oneNetMqttLoop()   — 维护 MQTT 连接、保活与自动重连
 * 4. updateOLEDDisplay() — 刷新 OLED 当前页面
 * 5. 每 500ms 采样一次重量，更新告警灯
 * 6. 每 10s 向 OneNET 上报一次物模型属性（仅 MQTT 已连接时）
 */
void loop()
{
  uint32_t now = millis();

  // 先处理来自摄像头的识别串口数据，保证消息不会堆积。
  pollCameraUart();
  // 上一步已经完成识别，根据识别结果旋转舵机角度
  if (g_lastAiDetected)
  {
    if (strcmp(g_lastAiLabel, "mobile-phone") == 0)
    {
      setServoAngle(0); // 手机仓
    }
    else if (strcmp(g_lastAiLabel, "Mouse") == 0)
    {
      setServoAngle(90); // 鼠标仓
    }
    else if (strcmp(g_lastAiLabel, "Battery") == 0)
    {
      setServoAngle(180); // 电池仓
    }
  }
  // 按键轮询，消抖并且是下降沿触发回调
  pollButtons();

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

    // 暂时三仓数据源统一为同一个 HX711
    // 百分比基于最大满溢重量1000g，满溢阈值 FULL_WEIGHT 独立判断
    float pct = (currentWeight * 100.0f) / 1000.0f;
    if (pct < 0.0f)
      pct = 0.0f;
    if (pct > 100.0f)
      pct = 100.0f;
    bool isFull = (currentWeight >= FULL_WEIGHT);

    BoxBinData binData = {currentWeight, pct, isFull};

    // OLED 上传状态置为 active
    setUploadingStatus(true);

    // 上传物模型属性：手机仓 / 鼠标仓 / 电池仓（当前同源）
    oneNetMqttUploadProperties(binData, binData, binData);

    // 这里额外输出最近一次 AI 状态，方便你在串口联调时观察。
    // if (g_lastAiDetected)
    // {
    //   Serial.printf("[AI-STATE] label=%s conf=%.3f age=%lums\n", g_lastAiLabel, g_lastAiConf, (unsigned long)(now - g_lastAiUpdateMs));
    // }
    // else
    // {
    //   Serial.printf("[AI-STATE] none age=%lums\n", (unsigned long)(now - g_lastAiUpdateMs));
    // }
  }
}
