#include "oledInit.h"
#include <WiFi.h>

Adafruit_SSD1306 oledDisplay(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// 页面状态变量
static uint8_t currentPage = 0;       // 当前页面：0=WiFi状态, 1=上传动画
static uint32_t lastPageSwitchMs = 0; // 上次页面切换时间
static bool uploadingActive = false;  // 是否正在上传数据
static uint8_t animationFrame = 0;    // 动画帧计数
static uint32_t lastAnimationMs = 0;  // 上次动画更新时间
#define ANIMATION_INTERVAL 300        // 动画更新间隔（毫秒）

// 设置上传状态
void setUploadingStatus(bool isUploading)
{
    uploadingActive = isUploading;
}

// 页面1：显示WiFi连接状态
static void showWiFiStatusPage()
{
    oledDisplay.clearDisplay();
    oledDisplay.setTextSize(1);
    oledDisplay.setTextColor(SSD1306_WHITE);
    oledDisplay.setCursor(0, 0);
    oledDisplay.println("=== WiFi Status ===");
    oledDisplay.println();

    if (WiFi.status() == WL_CONNECTED)
    {
        oledDisplay.setTextSize(2);
        oledDisplay.println("Connected");
        oledDisplay.setTextSize(1);
        oledDisplay.println();
        oledDisplay.printf("SSID: %s\n", WiFi.SSID().c_str());
        oledDisplay.printf("IP: %s\n", WiFi.localIP().toString().c_str());
        oledDisplay.printf("RSSI: %d dBm", WiFi.RSSI());
    }
    else
    {
        oledDisplay.setTextSize(2);
        oledDisplay.println("Disconnected");
        oledDisplay.setTextSize(1);
        oledDisplay.println();
        oledDisplay.println("WiFi not connected");
    }

    oledDisplay.display();
}

// 页面2：显示上传动画
static void showUploadPage()
{
    oledDisplay.clearDisplay();
    oledDisplay.setTextSize(1);
    oledDisplay.setTextColor(SSD1306_WHITE);
    oledDisplay.setCursor(0, 0);
    oledDisplay.println("=== Data Upload ===");
    oledDisplay.println();

    if (WiFi.status() == WL_CONNECTED && uploadingActive)
    {
        // 显示上传动画
        oledDisplay.setTextSize(2);
        oledDisplay.setCursor(10, 24);
        oledDisplay.print("Uploading");

        // 动态点动画
        String dots = "";
        for (int i = 0; i <= (animationFrame % 4); i++)
        {
            dots += ".";
        }
        oledDisplay.print(dots);

        // 绘制上传箭头动画
        int arrowY = 50 - (animationFrame % 3) * 4;
        oledDisplay.drawLine(64, arrowY + 8, 64, arrowY, SSD1306_WHITE); // 箭头竖线
        oledDisplay.drawLine(64, arrowY, 58, arrowY + 6, SSD1306_WHITE); // 左翼
        oledDisplay.drawLine(64, arrowY, 70, arrowY + 6, SSD1306_WHITE); // 右翼

        // 绘制云朵图标
        oledDisplay.fillCircle(100, 56, 6, SSD1306_WHITE);
        oledDisplay.fillCircle(108, 58, 5, SSD1306_WHITE);
        oledDisplay.fillCircle(116, 56, 6, SSD1306_WHITE);
        oledDisplay.fillRect(100, 56, 16, 8, SSD1306_WHITE);
    }
    else if (WiFi.status() != WL_CONNECTED)
    {
        oledDisplay.setTextSize(1);
        oledDisplay.setCursor(0, 24);
        oledDisplay.println("WiFi Disconnected");
        oledDisplay.println("Cannot upload data");
    }
    else
    {
        oledDisplay.setTextSize(1);
        oledDisplay.setCursor(0, 24);
        oledDisplay.println("Idle");
        oledDisplay.println("No data uploading");
    }

    oledDisplay.display();
}

// 更新OLED显示（页面切换和动画更新）
void updateOLEDDisplay()
{
    uint32_t now = millis();

    // 更新动画帧
    if (now - lastAnimationMs >= ANIMATION_INTERVAL)
    {
        lastAnimationMs = now;
        animationFrame++;
    }

    // 检查是否需要切换页面
    if (now - lastPageSwitchMs >= PAGE_SWITCH_INTERVAL)
    {
        lastPageSwitchMs = now;
        currentPage = (currentPage + 1) % 2; // 在两个页面间切换
    }

    // 显示当前页面
    switch (currentPage)
    {
    case 0:
        showWiFiStatusPage();
        break;
    case 1:
        showUploadPage();
        break;
    }
}

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

    // 初始化页面切换计时器
    lastPageSwitchMs = millis();
    lastAnimationMs = millis();
}