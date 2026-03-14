#include "connectToWiFi.h"
#include "oledInit.h"

/**
 * @brief 初始化 WiFi 并发起连接
 *
 * 说明：
 * 1. 以 WIFI_STA 模式连接由 connectToWiFi.h 中宏定义的 SSID/密码。
 * 2. 本函数不阻塞等待连接完成，连接状态由 main.cpp 的 setup()
 *    循环查询并推进进度动画，loop() 中由 OLED 状态页持续反映。
 * 3. 若调用时已连接，会打印详细网络信息（IP、SSID、RSSI、MAC）。
 */
void setupWiFi()
{
  Serial.println("Connecting to WiFi...");
  delay(10); // 短暂延时，保证串口输出稳定

  // 以“终端设备模式”连接路由器
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  // 这里保留了超时参数定义，方便后续扩展为阻塞等待连接。
  // 当前版本不阻塞，连接是否成功由主循环和状态页持续反映。
  int atttempts = 0;
  const int maxAttempts = 20;
  (void)atttempts;
  (void)maxAttempts;

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("WiFi connected successfully");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());
    Serial.print("Signal strength (RSSI): ");
    Serial.println(WiFi.RSSI());
    Serial.print("MAC address: ");
    Serial.println(WiFi.macAddress());
  }
  else
  {
    Serial.println("\nFailed to connect to WiFi");
  }
}
