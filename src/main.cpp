#include <Arduino.h>
#include <time.h>
#include "oledInit.h"
#include "connectToWiFi.h"
#include "onenetToken.h"

static const char *ONENET_PRODUCT_ID = "your_product_id";
static const char *ONENET_DEVICE_NAME = "your_device_name";
static const char *ONENET_BASE64_KEY = "your_base64_device_or_product_key";

static String buildDeviceResource(const char *productId, const char *deviceName)
{
  return String("products/") + productId + "/devices/" + deviceName;
}

void setup()
{
  Serial.begin(115200); // 初始化串口0通信
  setupOLED();          // 初始化 OLED 显示
  setupWiFi();          // 连接 WiFi

  String resource = buildDeviceResource(ONENET_PRODUCT_ID, ONENET_DEVICE_NAME);
  uint32_t expirationTime = static_cast<uint32_t>(time(nullptr)) + 3600;
  String token = generateOneNetToken(
      ONENET_BASE64_KEY,
      resource,
      expirationTime,
      OneNetSignMethod::SHA256,
      "2018-10-31");

  Serial.println("OneNET resource:");
  Serial.println(resource);
  Serial.println("OneNET token:");
  Serial.println(token);

  Serial.println("Setup complete");
}
void loop()
{
  // 主循环中可以添加其他功能
}