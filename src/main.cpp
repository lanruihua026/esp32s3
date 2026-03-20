#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "connectToWiFi.h"
#include "hx711.h"
#include "oledInit.h"
#include "onenetMqtt.h"
#include "buttonControl.h"
#include "servoControl.h"
// ===== 常量宏定义 =====
#define Warnlight 6             // 告警灯 GPIO
#define RGB_LED_PIN 38          // 板载 WS2812 RGB LED GPIO（ESP32-S3-DevKitM-1）
#define RGB_LED_NUM 1           // 板载只有 1 颗
#define FULL_WEIGHT 1000        // 实际满载阈值（g）：HX711量程5000g，设定最大载重1000g
#define HX711_CAL_FACTOR 449.1f // HX711 校准因子（raw/g）：以216g砝码校准（原始值1000×97/216≈449.1）

// 板载 RGB LED 驱动对象（上电即关闭，防止误点亮）
static Adafruit_NeoPixel boardRgb(RGB_LED_NUM, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

// ===== ESP32-CAM -> ESP32-S3 串口参数 =====
// 接线说明：
// 1) ESP32-CAM TX（1） -> ESP32-S3 RX（18）
// 2) ESP32-CAM RX （3）<- ESP32-S3 TX（17）
// 3) 两块板必须共地（GND 对 GND）
static const int CAM_UART_RX_PIN = 18;
static const int CAM_UART_TX_PIN = 17;
static const uint32_t CAM_UART_BAUD = 115200;

// 串口对象：用于接收 ESP32-CAM 输出的识别结果。
HardwareSerial CameraUart(1);

// 串口接收缓存：按“逐字符收、按行解析”策略处理。
static char g_camLineBuf[128] = {0};
static size_t g_camLinePos = 0;

// 最近一次模型识别结果缓存
static bool g_lastAiDetected = false;   // 是否识别到目标
static char g_lastAiLabel[32] = "none"; // 识别到的标签 默认"none" 即没有识别结果
static float g_lastAiConf = 0.0f;       // 结果置信度 (0.0~1.0)
static uint32_t g_lastAiUpdateMs = 0;   // 上次识别结果更新时间戳（ms），用于 OLED 显示过期状态

// ===== OneNET 设备身份 =====
// 说明：这些信息需要与你在 OneNET 平台创建的产品/设备保持一致。
static const char *ONENET_PRODUCT_ID = "f45hkc7xC7";
static const char *ONENET_DEVICE_NAME = "Box1";
static const char *ONENET_BASE64_KEY = "T0R5ejYyM1JrT2VuczBkZllINmZuazRicEMxc29xcnk=";

// 定时任务时间戳
static uint32_t lastReportMs = 0; // 上次上报属性时间
static uint32_t lastSampleMs = 0; // 上次采样重量时间
static uint32_t lastWiFiRetryMs = 0;

// 当前重量缓存（供 OLED 显示、告警判断、云上报复用）
static int32_t currentWeight = 0;

// 初始化与运行态健康状态缓存
static bool g_hx711InitOk = false;
static bool g_wifiInitTimeout = false;
static bool g_wifiEverConnected = false;

static void syncRuntimeHealth()
{
  bool wifiConnected = (WiFi.status() == WL_CONNECTED);
  if (wifiConnected)
  {
    g_wifiEverConnected = true;
    // 解决“启动阶段超时，稍后恢复成功”后状态仍显示错误的问题。
    g_wifiInitTimeout = false;
  }

  bool mqttConnected = oneNetMqttConnected();
  setRuntimeHealth(wifiConnected, g_wifiInitTimeout, g_hx711InitOk, mqttConnected);
}

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

    // Serial.printf("[CAM-UART] DET label=%s conf=%.3f\n", g_lastAiLabel, g_lastAiConf);
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

    // Serial.println("[CAM-UART] NONE");
    return;
  }

  // 3) 异常消息
  if (strncmp(line, "ERR,", 4) == 0)
  {
    g_lastAiDetected = false;
    g_lastAiUpdateMs = millis();

    // 异常时也更新 OLED 缓存
    setAiResult(false, "ERR", 0.0f, g_lastAiUpdateMs);
    // Serial.printf("[CAM-UART] %s\n", line);
    return;
  }

  // 4) 其他噪声行：打印出来便于调试，但不影响业务主流程。
  // Serial.printf("[CAM-UART] Unknown line: %s\n", line);
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

  // 关闭板载 RGB LED，防止上电随机点亮
  boardRgb.begin();
  boardRgb.setPixelColor(0, 0); // 黑色 = 关闭
  boardRgb.show();

  // 初始化来自 ESP32-CAM 的串口通道。
  CameraUart.begin(CAM_UART_BAUD, SERIAL_8N1, CAM_UART_RX_PIN, CAM_UART_TX_PIN);
  // Serial.printf("[CAM-UART] Init done, baud=%u RX=%d TX=%d\n", CAM_UART_BAUD, CAM_UART_RX_PIN, CAM_UART_TX_PIN);

  // 1) OLED 初始化，并显示开机进度框架
  resetInitModuleStatus();
  setInitModuleStatus(INIT_MODULE_OLED, INIT_RUNNING, "Init");
  setupOLED();
  setInitModuleStatus(INIT_MODULE_OLED, isOLEDReady() ? INIT_OK : INIT_ERROR, isOLEDReady() ? "Ready" : "Fail");
  setFullWeight(FULL_WEIGHT); // 将满载阈值传入 OLED 模块

  // 2) HX711 初始化与校准参数设置（20%~40%）
  setInitModuleStatus(INIT_MODULE_HX711, INIT_RUNNING, "Probe");
  showBootProgress(20, "HX711 Sensor");
  g_hx711InitOk = setupHX711();
  setInitModuleStatus(INIT_MODULE_HX711, g_hx711InitOk ? INIT_OK : INIT_ERROR, g_hx711InitOk ? "Ready" : "Timeout");
  delay(100);

  showBootProgress(30, "Calibration");
  setCalibrationFactor(HX711_CAL_FACTOR);
  delay(100);

  // 3) WiFi 连接（40%~70%，最多等待 10 秒后继续进入系统）
  setInitModuleStatus(INIT_MODULE_WIFI, INIT_RUNNING, "Connecting");
  showBootProgress(40, "WiFi Connecting");
  setupWiFi();

  const uint32_t wifiWaitStartMs = millis();
  const uint32_t wifiWaitWindowMs = 10000;
  while (WiFi.status() != WL_CONNECTED && (millis() - wifiWaitStartMs) < wifiWaitWindowMs)
  {
    delay(200);
    uint32_t elapsed = millis() - wifiWaitStartMs;
    uint8_t wifiProgress = 40 + (uint8_t)((elapsed * 25UL) / wifiWaitWindowMs);
    showBootProgress(wifiProgress, "WiFi Connecting");
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    g_wifiEverConnected = true;
    g_wifiInitTimeout = false;
    setInitModuleStatus(INIT_MODULE_WIFI, INIT_OK, "Connected");
    showBootProgress(70, "WiFi Connected");
  }
  else
  {
    g_wifiInitTimeout = true;
    setInitModuleStatus(INIT_MODULE_WIFI, INIT_TIMEOUT, "10s timeout");
    showBootProgress(70, "WiFi Timeout");
  }
  delay(200);

  // 舵机初始化放在 WiFi 之后，供电更稳定，避免 attach 后立刻 write 造成电流冲击
  setInitModuleStatus(INIT_MODULE_SERVO, INIT_RUNNING, "Attach");
  showBootProgress(75, "Servo Init");
  initServo();
  setServoAngle(0);
  setInitModuleStatus(INIT_MODULE_SERVO, INIT_OK, "Ready");

  // 4) 告警灯 GPIO 初始化（70%~80%）
  showBootProgress(78, "Warning Light");
  pinMode(Warnlight, OUTPUT);
  delay(100);

  // 5) 按键初始化，并注册回调
  setInitModuleStatus(INIT_MODULE_BUTTON, INIT_RUNNING, "IRQ");
  showBootProgress(86, "Buttons");
  setupButtons();
  setButton1Callback([]()
                     { toggleOledPage(); }); // 按键1：循环切换页面
  setInitModuleStatus(INIT_MODULE_BUTTON, INIT_OK, "Ready");
  delay(100);

  // 7) OneNET MQTT 初始化（86%~100%）
  // 注意：这里只做配置，真实连接在 loop() 内由 oneNetMqttLoop 自动完成。
  setInitModuleStatus(INIT_MODULE_MQTT, INIT_RUNNING, "Config");
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
  setInitModuleStatus(INIT_MODULE_MQTT, INIT_OK, "Ready");

  showBootProgress(95, "System Ready");
  delay(200);
  showBootProgress(100, "Starting...");
  delay(300);

  // 初始化定时器
  lastReportMs = millis();
  lastSampleMs = millis();
  lastWiFiRetryMs = millis();
  syncRuntimeHealth();
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
    if (strcmp(g_lastAiLabel, "MobilePhone") == 0)
    {
      setServoAngle(0); // 手机仓
    }
    else if (strcmp(g_lastAiLabel, "Charger") == 0)
    {
      setServoAngle(45); // 充电器仓
    }
    else if (strcmp(g_lastAiLabel, "Battery") == 0)
    {
      setServoAngle(90); // 电池仓
    }
    else if (strcmp(g_lastAiLabel, "Earphone") == 0)
    {
      setServoAngle(135); // 耳机仓
    }
    else
    {
      setServoAngle(0); // 默认角度
    }
  }
  else
  {
    setServoAngle(0); // 默认角度
  }
  // 按键轮询，消抖并且是下降沿触发回调
  pollButtons();

  // 若 WiFi 仍未连上，按固定周期主动重试一次，避免长期停留在离线状态。
  if (WiFi.status() != WL_CONNECTED && now - lastWiFiRetryMs >= 15000)
  {
    lastWiFiRetryMs = now;
    WiFi.disconnect(false, false);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }

  // 维护 MQTT 连接、收发和自动重连
  oneNetMqttLoop();

  // 同步 OLED 运行态健康状态（WiFi 后续成功会自动清除启动期超时错误）。
  syncRuntimeHealth();

  // 持续刷新 OLED 综合状态页
  updateOLEDDisplay();

  // 每 500ms 采样一次重量
  if (now - lastSampleMs >= 500)
  {
    lastSampleMs = now;
    currentWeight = (int32_t)getWeight();
    setCurrentWeight(currentWeight);

    // 满载点亮告警灯（含迟滞，避免阈值附近噪声导致闪烁）
    static bool s_warningOn = false;
    if (!s_warningOn && currentWeight >= FULL_WEIGHT)
    {
      s_warningOn = true;
      digitalWrite(Warnlight, HIGH);
    }
    else if (s_warningOn && currentWeight < FULL_WEIGHT - 100)
    {
      s_warningOn = false;
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

    // 上传物模型属性：手机仓 / 数码配件仓 / 电池仓（当前同源）
    bool uploadOk = oneNetMqttUploadProperties(binData, binData, binData);
    if (uploadOk)
    {
      Serial.println("[MQTT] Properties uploaded OK");
    }
  }
}
