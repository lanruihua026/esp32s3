#include <Arduino.h>

#include "connectToWiFi.h"
#include "hx711.h"
#include "oledInit.h"
#include "onenetMqtt.h"

// ===== 业务参数 =====
#define Warnlight 6               // 告警灯 GPIO
#define FULL_WEIGHT 400           // 满载阈值（当前按克处理）
#define HX711_CAL_FACTOR 1000.0f  // HX711 校准因子（raw/g）

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

void setup()
{
  Serial.begin(115200);

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
  }
}
