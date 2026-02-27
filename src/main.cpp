#include <Arduino.h>
#include "oledInit.h"
#include "connectToWiFi.h"
#include "onenetMqtt.h"

// ===== OneNET 设备身份信息 =====
// ONENET_PRODUCT_ID: 产品 ID（来自 OneNET 平台）
// ONENET_DEVICE_NAME: 设备名称（需与平台设备一致）
// ONENET_BASE64_KEY: 设备密钥（Base64 编码）
static const char *ONENET_PRODUCT_ID = "f45hkc7xC7";
static const char *ONENET_DEVICE_NAME = "Box1";
static const char *ONENET_BASE64_KEY = "T0R5ejYyM1JrT2VuczBkZllINmZuazRicEMxc29xcnk=";

// 上报节流时间戳（毫秒）
static uint32_t lastReportMs = 0;

// 演示用模拟重量值。
// 实际项目中可替换为 HX711 等称重传感器读数。
static int32_t mockWeight = 10;

void setup()
{
  Serial.begin(115200); // 初始化串口通信，方便调试输出
  setupOLED();          // 初始化 OLED 显示
  setupWiFi();          // 连接 WiFi 网络

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
  Serial.println("Setup complete");
}

void loop()
{
  // 维持 MQTT 心跳、接收消息、自动重连。
  oneNetMqttLoop();

  // 更新OLED显示（页面切换）
  updateOLEDDisplay();

  // 每 10 秒上报一次属性。
  uint32_t now = millis();
  if (oneNetMqttConnected() && now - lastReportMs >= 10000)
  {
    lastReportMs = now;

    // 演示数据：重量每次 +15，超过 500 后回到 0。
    mockWeight += 15;
    if (mockWeight > 500)
    {
      mockWeight = 0;
    }

    // 业务规则示例：重量 >= 400kg 认为仓格已满。
    bool isFull = (mockWeight >= 400);

    // 设置上传状态为true，触发上传动画
    setUploadingStatus(true);

    // 按 OneJSON 物模型格式上报 isFull 与 weight。
    oneNetMqttUploadProperties(isFull, mockWeight);
  }
}