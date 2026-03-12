#include "oledInit.h"
#include <WiFi.h>

Adafruit_SSD1306 oledDisplay(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ===== 运行态显示所需共享状态 =====
static bool uploadingActive = false; // 是否正在上传
static int32_t currentWeight = 0;    // 当前重量（g）
static int32_t fullWeight = 1000;    // 满载阈值（g），由 main.cpp 通过 setFullWeight() 初始化

// 动画帧计数器（用于 Upload... 点动画）
static uint8_t animationFrame = 0;
static uint32_t lastAnimationMs = 0;
#define ANIMATION_INTERVAL 300 // 动画刷新周期（ms）

// ===== OLED 页面管理 =====
// 0 = 综合信息页；1 = AI 识别结果页
static uint8_t g_oledPage = 0;

// ===== AI 识别结果缓存 =====
static bool g_aiDetected = false;
static char g_aiLabel[32] = "none";
static float g_aiConf = 0.0f;
static uint32_t g_aiUpdateMs = 0;

/**
 * @brief 设置上传状态
 * @param isUploading true 表示正在上传，false 表示空闲
 * 说明：这个状态会在 OLED 上以动画形式展示，提升用户反馈体验
 */
void setUploadingStatus(bool isUploading)
{
    uploadingActive = isUploading;
}

/**
 * @brief 设置当前重量
 * @param weight 当前重量（单位：克）
 * 说明：这个函数会更新全局重量状态，供 OLED 显示、告警判断和云上报复用。
 */
void setCurrentWeight(int32_t weight)
{
    currentWeight = weight;
}

void setFullWeight(int32_t fw)
{
    if (fw > 0)
        fullWeight = fw;
}

void setAiResult(bool detected, const char *label, float conf, uint32_t updateMs)
{
    g_aiDetected = detected;
    if (label)
    {
        strncpy(g_aiLabel, label, sizeof(g_aiLabel) - 1);
        g_aiLabel[sizeof(g_aiLabel) - 1] = '\0';
    }
    g_aiConf = conf;
    g_aiUpdateMs = updateMs;
}

void setOledPage(uint8_t page)
{
    g_oledPage = page;
}

/**
 * @brief 显示 AI 识别结果页
 *
 * 页面内容：
 * 1. 页面标题
 * 2. 识别状态（Detected / No Target）
 * 3. 识别标签
 * 4. 置信度
 * 5. 上次更新距今时间
 */
static void showAiResultPage()
{
    oledDisplay.clearDisplay();
    oledDisplay.setTextSize(1);
    oledDisplay.setTextColor(SSD1306_WHITE);

    // 标题行
    oledDisplay.setCursor(0, 0);
    oledDisplay.println("==== AI Result ====");

    if (g_aiDetected)
    {
        // 反显 Detected 状态，醒目显示识别成功
        oledDisplay.setCursor(0, 11);
        oledDisplay.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        oledDisplay.println(" Status: Detected ");
        oledDisplay.setTextColor(SSD1306_WHITE);

        // 识别标签（最多显示 20 个字符）
        char labelBuf[21];
        strncpy(labelBuf, g_aiLabel, 20);
        labelBuf[20] = '\0';
        oledDisplay.setCursor(0, 23);
        oledDisplay.print("Label: ");
        oledDisplay.println(labelBuf);

        // 置信度
        char confBuf[20];
        snprintf(confBuf, sizeof(confBuf), "Conf:  %.1f%%", g_aiConf * 100.0f);
        oledDisplay.setCursor(0, 34);
        oledDisplay.println(confBuf);
    }
    else
    {
        oledDisplay.setCursor(0, 11);
        oledDisplay.println("Status: No Target");
        oledDisplay.setCursor(0, 23);
        oledDisplay.println("Label: --");
        oledDisplay.setCursor(0, 34);
        oledDisplay.println("Conf:  --");
    }

    // 上次更新距今时间
    oledDisplay.setCursor(0, 45);
    if (g_aiUpdateMs == 0)
    {
        oledDisplay.println("No data yet");
    }
    else
    {
        uint32_t ageSec = (millis() - g_aiUpdateMs) / 1000;
        char ageBuf[24];
        snprintf(ageBuf, sizeof(ageBuf), "Updated: %lus ago", (unsigned long)ageSec);
        oledDisplay.println(ageBuf);
    }

    // 底部提示当前页
    oledDisplay.setCursor(0, 56);
    oledDisplay.println("[Btn2: Info Page]");

    oledDisplay.display();
}

/**
 * @brief 显示综合状态页
 *
 * 页面内容：
 * 1. WiFi 在线状态
 * 2. 当前重量
 * 3. 满载状态
 * 4. 上传状态（含动画）
 */
static void showCombinedPage()
{
    oledDisplay.clearDisplay();
    oledDisplay.setTextSize(1);
    oledDisplay.setTextColor(SSD1306_WHITE);

    oledDisplay.setCursor(0, 0);
    oledDisplay.println("====== STATUS ======");

    oledDisplay.setCursor(0, 11);
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
        // 反显 Offline，提高离线状态可读性
        oledDisplay.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        oledDisplay.println("Offline");
        oledDisplay.setTextColor(SSD1306_WHITE);
    }

    // 计算重量百分比（以 fullWeight 为 100% 基准）
    float loadPct = (currentWeight * 100.0f) / fullWeight;
    if (loadPct < 0.0f)
        loadPct = 0.0f;
    if (loadPct > 100.0f)
        loadPct = 100.0f;

    oledDisplay.setCursor(0, 22);
    oledDisplay.print("Weight: ");
    oledDisplay.print(currentWeight);
    oledDisplay.println(" g");

    // 显示负载百分比
    char pctBuf[20];
    snprintf(pctBuf, sizeof(pctBuf), "Load:  %.2f%%", loadPct);
    oledDisplay.setCursor(0, 33);
    oledDisplay.println(pctBuf);

    oledDisplay.setCursor(0, 44);
    if (currentWeight >= fullWeight)
    {
        // 超阈值反显提示
        oledDisplay.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        oledDisplay.println("  Status: FULL   ");
    }
    else
    {
        oledDisplay.setTextColor(SSD1306_WHITE);
        oledDisplay.println("Status: Normal");
    }
    oledDisplay.setTextColor(SSD1306_WHITE);

    oledDisplay.setCursor(0, 44);
    if (currentWeight >= fullWeight)
    {
        // 超阈值反显提示
        oledDisplay.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        oledDisplay.println("  Status: FULL   ");
    }
    else
    {
        oledDisplay.setTextColor(SSD1306_WHITE);
        oledDisplay.println("Status: Normal");
    }
    oledDisplay.setTextColor(SSD1306_WHITE);

    oledDisplay.setCursor(0, 55);
    if (WiFi.status() == WL_CONNECTED)
    {
        if (uploadingActive)
        {
            oledDisplay.print("Upload:");
            // 简单点动画：., .., ..., ....
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

void updateOLEDDisplay()
{
    uint32_t now = millis();

    // 动画按固定周期推进，不依赖上传频率
    if (now - lastAnimationMs >= ANIMATION_INTERVAL)
    {
        lastAnimationMs = now;
        animationFrame++;
    }

    // 根据当前选中页面分支显示
    if (g_oledPage == 1)
    {
        showAiResultPage();
    }
    else
    {
        showCombinedPage();
    }
}

void showBootProgress(uint8_t progress, const char *statusText)
{
    const uint8_t barX = 10;
    const uint8_t barY = 40;
    const uint8_t barWidth = 108;
    const uint8_t barHeight = 12;

    oledDisplay.clearDisplay();
    oledDisplay.setTextSize(1);
    oledDisplay.setTextColor(SSD1306_WHITE);

    oledDisplay.setCursor(25, 0);
    oledDisplay.println("System Boot");

    oledDisplay.setCursor(0, 16);
    oledDisplay.println("Initializing...");
    oledDisplay.setCursor(0, 28);

    // 为防止越界，仅显示前 16 个字符
    char displayText[17];
    strncpy(displayText, statusText, 16);
    displayText[16] = '\0';
    oledDisplay.print("> ");
    oledDisplay.println(displayText);

    // 进度条外框
    oledDisplay.drawRect(barX, barY, barWidth, barHeight, SSD1306_WHITE);

    // 进度条填充（留 2 像素边框）
    uint8_t fillWidth = (uint8_t)((progress * (barWidth - 4)) / 100);
    if (fillWidth > 0)
    {
        oledDisplay.fillRect(barX + 2, barY + 2, fillWidth, barHeight - 4, SSD1306_WHITE);
    }

    oledDisplay.setCursor(50, 54);
    oledDisplay.print(progress);
    oledDisplay.println("%");

    oledDisplay.display();
}

void setupOLED()
{
    // 指定 I2C 引脚与总线速率（400kHz）
    Wire.begin(SDA_PIN, SCL_PIN, 400000);

    // OLED 初始化失败时直接停机，避免后续状态不可见
    if (!oledDisplay.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
    {
        Serial.println("SSD1306 init failed");
        while (true)
        {
            delay(1000);
        }
    }

    showBootProgress(0, "OLED Ready");
    lastAnimationMs = millis();
}
