#include "oledInit.h"
#include <WiFi.h>

Adafruit_SSD1306 oledDisplay(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// 显示状态变量
static bool uploadingActive = false; // 是否正在上传数据
static int32_t currentWeight = 0;    // 当前重量值
static uint8_t animationFrame = 0;   // 动画帧计数
static uint32_t lastAnimationMs = 0; // 上次动画更新时间
#define ANIMATION_INTERVAL 300       // 动画更新间隔（毫秒）

// 设置上传状态
void setUploadingStatus(bool isUploading)
{
    uploadingActive = isUploading;
}

// 设置当前重量值
void setCurrentWeight(int32_t weight)
{
    currentWeight = weight;
}

// 显示综合信息页面：WiFi状态 + 上传状态 + 重量
static void showCombinedPage()
{
    oledDisplay.clearDisplay();
    oledDisplay.setTextSize(1);
    oledDisplay.setTextColor(SSD1306_WHITE);

    // 第1行：标题
    oledDisplay.setCursor(0, 0);
    oledDisplay.println("=== Smart Bin ===");

    // 第2行：WiFi状态
    oledDisplay.setCursor(0, 12);
    if (WiFi.status() == WL_CONNECTED)
    {
        oledDisplay.print("WiFi: ");
        oledDisplay.setTextColor(SSD1306_WHITE);
        oledDisplay.println("Online");
    }
    else
    {
        oledDisplay.setTextColor(SSD1306_WHITE);
        oledDisplay.print("WiFi: ");
        oledDisplay.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // 反显
        oledDisplay.println("Offline");
        oledDisplay.setTextColor(SSD1306_WHITE);
    }

    // 第3行：重量显示
    oledDisplay.setCursor(0, 24);
    oledDisplay.print("Weight: ");
    oledDisplay.print(currentWeight);
    oledDisplay.println(" g");

    // 第4行：满溢状态
    oledDisplay.setCursor(0, 36);
    if (currentWeight >= 400)
    {
        oledDisplay.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // 反显表示警告
        oledDisplay.println("  Status: FULL   ");
    }
    else
    {
        oledDisplay.setTextColor(SSD1306_WHITE);
        oledDisplay.println("  Status: Normal ");
    }
    oledDisplay.setTextColor(SSD1306_WHITE);

    // 第5-6行：上传状态
    oledDisplay.setCursor(0, 48);
    if (WiFi.status() == WL_CONNECTED)
    {
        if (uploadingActive)
        {
            oledDisplay.print("Upload:");
            // 动态点动画
            for (int i = 0; i <= (animationFrame % 4); i++)
            {
                oledDisplay.print(".");
            }
            oledDisplay.print(" OK");
        }
        else
        {
            oledDisplay.print("Upload: Idle");
        }
    }
    else
    {
        oledDisplay.print("Upload: No WiFi");
    }

    oledDisplay.display();
}

// 更新OLED显示
void updateOLEDDisplay()
{
    uint32_t now = millis();

    // 更新动画帧（仅用于上传动画的点号）
    if (now - lastAnimationMs >= ANIMATION_INTERVAL)
    {
        lastAnimationMs = now;
        animationFrame++;
    }

    // 显示综合页面
    showCombinedPage();
}

// 显示初始化进度条
void showBootProgress(uint8_t progress, const char *statusText)
{
    const uint8_t barX = 10;        // 进度条起始X坐标
    const uint8_t barY = 40;        // 进度条Y坐标
    const uint8_t barWidth = 108;   // 进度条总宽度
    const uint8_t barHeight = 12;   // 进度条高度

    oledDisplay.clearDisplay();
    oledDisplay.setTextSize(1);
    oledDisplay.setTextColor(SSD1306_WHITE);

    // 第1行：标题
    oledDisplay.setCursor(25, 0);
    oledDisplay.println("System Boot");

    // 第2-3行：当前初始化状态
    oledDisplay.setCursor(0, 16);
    oledDisplay.println("Initializing...");
    oledDisplay.setCursor(0, 28);
    // 显示当前状态文本（最多16个字符）
    char displayText[17];
    strncpy(displayText, statusText, 16);
    displayText[16] = '\0';
    oledDisplay.print("> ");
    oledDisplay.println(displayText);

    // 绘制进度条外框
    oledDisplay.drawRect(barX, barY, barWidth, barHeight, SSD1306_WHITE);

    // 计算并绘制进度填充
    uint8_t fillWidth = (uint8_t)((progress * (barWidth - 4)) / 100);
    if (fillWidth > 0)
    {
        oledDisplay.fillRect(barX + 2, barY + 2, fillWidth, barHeight - 4, SSD1306_WHITE);
    }

    // 显示进度百分比
    oledDisplay.setCursor(50, 54);
    oledDisplay.print(progress);
    oledDisplay.println("%");

    oledDisplay.display();
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

    // 显示开机动画框架
    showBootProgress(0, "OLED Ready");

    // 初始化动画计时器
    lastAnimationMs = millis();
}