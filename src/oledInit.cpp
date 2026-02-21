#include "oledInit.h"

Adafruit_SSD1306 oledDisplay(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setupOLED()
{
    // 初始化 I2C：指定 SDA/SCL 引脚和 400kHz 速率
    Wire.begin(SDA_PIN, SCL_PIN, 400000);
    // 初始化 OLED，失败则停在此处
    if (!oledDisplay.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
    {
        Serial.println("SSD1306 init failed");
        while (true)
        {
            delay(1000);
        }
    }

    // 显示开机与硬件信息
    oledDisplay.clearDisplay();
    oledDisplay.setTextSize(1);
    oledDisplay.setTextColor(SSD1306_WHITE);
    oledDisplay.setCursor(0, 0);
    oledDisplay.println("Hello ESP32!");
    oledDisplay.println("SSD1306 OK");
    oledDisplay.printf("SDA:%d SCL:%d\n", SDA_PIN, SCL_PIN);
    oledDisplay.display();
}