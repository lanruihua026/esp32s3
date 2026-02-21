#include "connectToWiFi.h"
#include "oledInit.h"
void setupWiFi()
{
  Serial.println("Connecting to WiFi...");
  delay(10);                            // 稍作延迟以确保串口输出稳定
  WiFi.mode(WIFI_STA);                  // 设置为 STA 模式
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD); // 连接到指定的 WiFi 网络

  // WiFi超时等待，避免无限循环
  int atttempts = 0;          // 尝试连接次数
  const int maxAttempts = 20; // 最大尝试次数,每次尝试间隔1秒，总共等待20秒
  oledDisplay.clearDisplay();
  oledDisplay.setTextSize(1);
  oledDisplay.setTextColor(SSD1306_WHITE);
  oledDisplay.setCursor(0, 0);
  oledDisplay.println("Connecting to WiFi...");
  oledDisplay.display();
  while (WiFi.status() != WL_CONNECTED && atttempts < maxAttempts)
  {
    delay(1000); // 等待1秒钟
    Serial.print(".");
    oledDisplay.print(".");
    oledDisplay.display();
    atttempts++;
  }

  //  wifi连接成功或失败后，输出结果并在OLED上显示
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("WiFi connected successfully");
    Serial.print("IP address: "); // 输出连接成功后的IP地址
    Serial.println(WiFi.localIP());
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());              // 输出连接成功后的SSID
    Serial.print("Signal strength (RSSI): "); // 输出连接成功后的信号强度
    Serial.println(WiFi.RSSI());
    Serial.print("MAC address: ");
    Serial.println(WiFi.macAddress());
    oledDisplay.clearDisplay();
    oledDisplay.setTextSize(1);
    oledDisplay.setTextColor(SSD1306_WHITE);
    oledDisplay.setCursor(0, 0);
    oledDisplay.println("WiFi connected successfully");
    oledDisplay.printf("IP: %s\n", WiFi.localIP().toString().c_str());
    oledDisplay.printf("SSID: %s\n", WiFi.SSID().c_str());
    oledDisplay.printf("RSSI: %d dBm\n", WiFi.RSSI());
    oledDisplay.printf("MAC: %s\n", WiFi.macAddress().c_str());
    oledDisplay.display();
  }
  else
  {
    Serial.println("\nFailed to connect to WiFi");
    oledDisplay.clearDisplay();
    oledDisplay.setTextSize(1);
    oledDisplay.setTextColor(SSD1306_WHITE);
    oledDisplay.setCursor(0, 0);
    oledDisplay.println("WiFi Connection Failed");
    oledDisplay.display();
  }
}
