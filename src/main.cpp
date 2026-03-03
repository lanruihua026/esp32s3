#include <Arduino.h>
#include "oledInit.h"
#include "connectToWiFi.h"
#include "onenetMqtt.h"
#include "hx711.h"

// ====部分宏定义====
#define Warnlight 6              // 警示灯 GPIO 引脚
#define FULL_WEIGHT 400          // 满溢阈值（克）
#define HX711_CAL_FACTOR 1000.0f // HX711 校准因子（无砝码时可先调此值）

// ===== OneNET 设备身份信息 =====
// ONENET_PRODUCT_ID: 产品 ID（来自 OneNET 平台）
// ONENET_DEVICE_NAME: 设备名称（需与平台设备一致）
// ONENET_BASE64_KEY: 设备密钥（Base64 编码）
static const char *ONENET_PRODUCT_ID = "f45hkc7xC7";
static const char *ONENET_DEVICE_NAME = "Box1";
static const char *ONENET_BASE64_KEY = "T0R5ejYyM1JrT2VuczBkZllINmZuazRicEMxc29xcnk=";

// 上报节流时间戳（毫秒）
static uint32_t lastReportMs = 0;
// 称重采样节流时间戳（毫秒）
static uint32_t lastSampleMs = 0;

// 当前真实重量值
static int32_t currentWeight = 0;

void setup()
{
  Serial.begin(115200); // 初始化串口通信，方便调试输出
  setupOLED();          // 初始化 OLED 显示
  setupHX711();         // 初始化 HX711 称重传感器
  setCalibrationFactor(HX711_CAL_FACTOR);
  setupWiFi();                // 连接 WiFi 网络
  pinMode(Warnlight, OUTPUT); // 设置警示灯引脚为输出模式

  // 配置 OneNET MQTT 连接参数。
  // tokenExpireAt 示例设置为较远未来时间，避免短期内过期。
  OneNetMqttConfig cfg = {
      "mqtts.heclouds.com",
      1883,
      ONENET_PRODUCT_ID,
      ONENET_DEVICE_NAME,
      ONENET_BASE64_KEY,
      1893456000,
      OneNetSignMethod::SHA256};

  // 初始化 OneNET MQTT 模块（内部会在 loop 中自动尝试连接）。
  oneNetMqttBegin(cfg);

  lastReportMs = millis();
  lastSampleMs = millis();
  Serial.println("Setup complete");
}

void loop()
{
  uint32_t now = millis();

  // 维持 MQTT 心跳、接收消息、自动重连。
  oneNetMqttLoop();

  // 更新OLED显示（综合信息页面）
  updateOLEDDisplay();

  // 每 500ms 读取一次真实重量，供本地显示与告警使用。
  if (now - lastSampleMs >= 500)
  {
    lastSampleMs = now;
    currentWeight = (int32_t)getWeight();
    setCurrentWeight(currentWeight);

    if (currentWeight >= FULL_WEIGHT)
    {
      digitalWrite(Warnlight, HIGH);
    }
    else
    {
      digitalWrite(Warnlight, LOW);
    }
  }

  // 每 10 秒上报一次属性。
  if (oneNetMqttConnected() && now - lastReportMs >= 10000)
  {
    lastReportMs = now;

    // 业务规则：重量 >= FULL_WEIGHT 认为仓格已满
    bool isFull = (currentWeight >= FULL_WEIGHT);

    // 设置上传状态为true，触发上传动画
    setUploadingStatus(true);

    // 按 OneJSON 物模型格式上报 isFull 与 weight
    oneNetMqttUploadProperties(isFull, currentWeight);
  }
}