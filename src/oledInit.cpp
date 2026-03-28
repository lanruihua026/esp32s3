#include "oledInit.h"
#include <WiFi.h>
#include <cstring>

Adafruit_SH1106G oledDisplay(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// OLED 模块负责两类显示：
// 1. 开机初始化进度；
// 2. 运行过程中的系统状态、三仓重量、AI 识别结果。
static bool g_oledReady = false;

// ===== 运行态显示所需共享状态 =====
static int32_t currentWeight1 = 0; // 1号仓当前重量（g）
static int32_t currentWeight2 = 0; // 2号仓当前重量（g）
static int32_t currentWeight3 = 0; // 3号仓当前重量（g）
static int32_t fullWeight = 1000;  // 满载阈值（g），由 main.cpp 通过 setFullWeight() 初始化

// Dirty flag 机制：
// 只有数据变化时才重绘，避免 OLED 不停清屏导致闪烁。
static bool g_displayDirty = true; // 初始强制绘制一次

// ===== OLED 页面管理 =====
// 0 = 系统状态页；1 = 三仓重量页；2 = 识别结果页
static uint8_t g_oledPage = 0;

// ===== AI 识别结果缓存 =====
static bool g_aiDetected = false;
static char g_aiLabel[32] = "none";
static float g_aiConf = 0.0f;
static uint32_t g_aiUpdateMs = 0;

// ===== 启动阶段状态缓存 =====
static InitModuleStatus g_initStatus[INIT_MODULE_COUNT] = {INIT_PENDING};
static char g_initDetail[INIT_MODULE_COUNT][17] = {{0}};

// ===== 运行态模块健康状态 =====
static bool g_wifiOk = false;
static bool g_wifiInitTimeout = false;
static bool g_hx711Ok = false;
static bool g_mqttOk = false;

static const char *moduleStatusText(InitModuleStatus status)
{
    switch (status)
    {
    case INIT_RUNNING:
        return "RUN";
    case INIT_OK:
        return "OK";
    case INIT_ERROR:
        return "ERR";
    case INIT_TIMEOUT:
        return "TIME";
    case INIT_SKIPPED:
        return "SKIP";
    case INIT_PENDING:
    default:
        return "PEND";
    }
}

void setRuntimeHealth(bool wifiOk, bool wifiInitTimeout, bool hx711Ok, bool mqttOk)
{
    // 主页面显示的是“运行状态摘要”，这里只负责同步缓存。
    if (g_wifiOk != wifiOk || g_wifiInitTimeout != wifiInitTimeout || g_hx711Ok != hx711Ok || g_mqttOk != mqttOk)
    {
        g_wifiOk = wifiOk;
        g_wifiInitTimeout = wifiInitTimeout;
        g_hx711Ok = hx711Ok;
        g_mqttOk = mqttOk;
        g_displayDirty = true;
    }
}

/**
 * @brief 设置当前重量
 * @param weight 当前重量（单位：克）
 * 说明：这个函数会更新全局重量状态，供 OLED 显示、告警判断和云上报复用。
 */
void setCurrentWeight(int32_t weight)
{
    if (currentWeight1 != weight)
    {
        currentWeight1 = weight;
        g_displayDirty = true;
    }
}

void setCurrentWeights(int32_t weight1, int32_t weight2, int32_t weight3)
{
    if (currentWeight1 != weight1 || currentWeight2 != weight2 || currentWeight3 != weight3)
    {
        currentWeight1 = weight1;
        currentWeight2 = weight2;
        currentWeight3 = weight3;
        g_displayDirty = true;
    }
}

/**
 * @brief 设置满载阈值
 * @param fw 满载重量（单位：克，必须 > 0，否则忽略）
 * 说明：该值作为重量百分比和满溢判断的基准，由 main.cpp 在初始化阶段传入。
 */
void setFullWeight(int32_t fw)
{
    if (fw > 0)
        fullWeight = fw;
}

/**
 * @brief 更新 AI 识别结果缓存
 * @param detected   是否检测到目标
 * @param label      识别标签字符串（nullptr 时跳过赋值）
 * @param conf       置信度（0.0~1.0）
 * @param updateMs   本次更新的时间戳（millis()）
 * 说明：由 main.cpp 的 parseCameraLine() 在收到 CAM 消息后调用，
 * 将最新识别结果同步给 OLED AI 结果页。
 */
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
    g_displayDirty = true;
}

/**
 * @brief 切换 OLED 显示页面
 * @param page 目标页面编号（0 = 系统状态页；1 = 三仓重量页；2 = 识别结果页）
 * 说明：由按键回调在 main.cpp 中调用，实现页面切换。
 */
void setOledPage(uint8_t page)
{
    uint8_t normalized = page % 3;
    if (g_oledPage != normalized)
    {
        g_oledPage = normalized;
        g_displayDirty = true;
    }
}

/**
 * @brief 循环切换 OLED 显示页面
 * 说明：每次调用按 0→1→2→0 循环切换。
 */
void toggleOledPage()
{
    g_oledPage = (g_oledPage + 1) % 3;
    g_displayDirty = true;
}

void prevOledPage()
{
    // 与 toggleOledPage() 相反，向左翻页。
    g_oledPage = (g_oledPage + 2) % 3;
    g_displayDirty = true;
}

void resetInitModuleStatus()
{
    // 系统重新启动时，所有模块状态先回到“等待初始化”。
    for (uint8_t i = 0; i < INIT_MODULE_COUNT; ++i)
    {
        g_initStatus[i] = INIT_PENDING;
        g_initDetail[i][0] = '\0';
    }
}

void setInitModuleStatus(InitModuleId module, InitModuleStatus status, const char *detail)
{
    // setup() 每完成一个模块初始化，就会调用这里刷新开机进度页。
    if (module >= INIT_MODULE_COUNT)
    {
        return;
    }
    g_initStatus[module] = status;
    if (detail != nullptr)
    {
        strncpy(g_initDetail[module], detail, sizeof(g_initDetail[module]) - 1);
        g_initDetail[module][sizeof(g_initDetail[module]) - 1] = '\0';
    }
}

/**
 * @brief 显示模型识别结果页
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
    oledDisplay.setTextColor(SH110X_WHITE);

    // 标题行
    oledDisplay.setCursor(0, 0);
    oledDisplay.println("===Detected Result===");

    // 最近 3 秒内识别过目标时，页面继续保留“Detected”状态，
    // 避免摄像头偶发一帧没识别到就把结果立刻清掉。
    static const uint32_t AI_PERSIST_MS = 3000;
    bool showDetected = g_aiDetected ||
                        (g_aiUpdateMs > 0 &&
                         (millis() - g_aiUpdateMs) < AI_PERSIST_MS &&
                         strcmp(g_aiLabel, "none") != 0);

    if (showDetected)
    {
        // 反显 Detected 状态，醒目显示识别成功
        oledDisplay.setCursor(0, 11);
        oledDisplay.setTextColor(SH110X_BLACK, SH110X_WHITE);
        oledDisplay.println(" Status: Detected ");
        oledDisplay.setTextColor(SH110X_WHITE);

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

    // 显示“距离上次识别过了多久”，便于判断当前结果是不是旧数据。
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
    oledDisplay.display();
}

/**
 * @brief 显示三仓重量页
 *
 * 页面内容：
 * 1. 三个仓格重量（g）
 * 2. 三个仓格百分比（以 fullWeight 为 100%）
 * 3. 三个仓格满溢状态
 */
static void showBinWeightPage()
{
    if (!g_oledReady)
    {
        return;
    }

    oledDisplay.clearDisplay();
    oledDisplay.setTextSize(1);
    oledDisplay.setTextColor(SH110X_WHITE);

    oledDisplay.setCursor(0, 0);
    oledDisplay.println("=== Bin Weights ===");

    // 三个仓位逐行显示，格式统一为：
    // 仓位编号 + 当前重量 + 百分比 + 是否满载。
    const int32_t weights[3] = {currentWeight1, currentWeight2, currentWeight3};
    const uint8_t linesY[3] = {14, 28, 42};

    for (uint8_t i = 0; i < 3; ++i)
    {
        float pct = (weights[i] * 100.0f) / fullWeight;
        if (pct < 0.0f)
            pct = 0.0f;
        if (pct > 100.0f)
            pct = 100.0f;
        bool isFull = (weights[i] >= fullWeight);

        char lineBuf[25];
        snprintf(lineBuf, sizeof(lineBuf), "B%u:%4ldg %3u%% %s", i + 1,
                 (long)weights[i], (unsigned int)(pct + 0.5f), isFull ? "FULL" : "OK");

        oledDisplay.setCursor(0, linesY[i]);
        oledDisplay.println(lineBuf);
    }

    oledDisplay.setCursor(0, 56);
    // 显示当前满载阈值，方便调试和确认网页端下发的阈值是否已生效
    char limBuf[18];
    snprintf(limBuf, sizeof(limBuf), "Lim:%4ldg  <BTN>", (long)fullWeight);
    oledDisplay.println(limBuf);

    oledDisplay.display();
}

static void showSystemStatusPage()
{
    if (!g_oledReady)
    {
        return;
    }

    oledDisplay.clearDisplay();
    oledDisplay.setTextSize(1);
    oledDisplay.setTextColor(SH110X_WHITE);

    oledDisplay.setCursor(0, 0);
    oledDisplay.println("=== System Status ===");

    // 这页主要给调试和答辩演示用，快速看出网络、称重、云连接是否正常。
    oledDisplay.setCursor(0, 11);
    oledDisplay.print("WiFi : ");
    oledDisplay.println(g_wifiOk ? "OK" : (g_wifiInitTimeout ? "ERROR" : "PENDING"));

    oledDisplay.setCursor(0, 22);
    oledDisplay.print("HX711: ");
    oledDisplay.println(g_hx711Ok ? "OK" : "ERROR");

    oledDisplay.setCursor(0, 33);
    oledDisplay.print("MQTT : ");
    oledDisplay.println(g_mqttOk ? "OK" : "OFFLINE");

    oledDisplay.setCursor(0, 44);
    oledDisplay.print("OLED : ");
    oledDisplay.println(g_oledReady ? "OK" : "ERROR");

    oledDisplay.setCursor(0, 55);
    // 显示当前满载阈值，方便查看网页端下发命令是否已生效
    char limBuf[18];
    snprintf(limBuf, sizeof(limBuf), "Limit:%5ldg", (long)fullWeight);
    oledDisplay.print(limBuf);

    oledDisplay.display();
}

/**
 * @brief 刷新 OLED 显示内容（在 loop() 中持续调用）
 *
 * 说明：
 * 1. 判断当前是否真的需要刷新页面。
 * 2. 根据当前页号选择显示系统状态、重量或识别结果。
 * 3. 本函数无阻塞，适合在 loop() 中持续调用。
 */
void updateOLEDDisplay()
{
    if (!g_oledReady)
    {
        return;
    }

    // AI 识别结果页在“结果保留倒计时”阶段需要持续刷新，
    // 其他页面继续依赖 dirty flag 节流即可。
    static const uint32_t AI_PERSIST_MS = 3000;
    bool inAiPersist = (g_oledPage == 2) &&
                       !g_aiDetected &&
                       (g_aiUpdateMs > 0) &&
                       ((millis() - g_aiUpdateMs) < AI_PERSIST_MS) &&
                       (strcmp(g_aiLabel, "none") != 0);

    if (!g_displayDirty && !inAiPersist)
    {
        return;
    }
    g_displayDirty = false;

    // 根据当前页号切换显示内容。
    if (g_oledPage == 0)
    {
        showSystemStatusPage();
    }
    else if (g_oledPage == 1)
    {
        showBinWeightPage();
    }
    else
    {
        showAiResultPage();
    }
}
/**
 * @brief 显示系统启动进度
 * @param progress 进度百分比（0~100）
 * @param statusText 当前阶段文本
 * 业务含义：
 * 用户上电后能直接看到系统现在初始化到了哪一步，
 * 比如在连 WiFi、初始化舵机还是配置 MQTT。
 */
void showBootProgress(uint8_t progress, const char *statusText)
{
    if (!g_oledReady)
    {
        return;
    }

    const uint8_t barX = 10;
    const uint8_t barY = 34;
    const uint8_t barWidth = 108;
    const uint8_t barHeight = 10;

    oledDisplay.clearDisplay();
    oledDisplay.setTextSize(1);
    oledDisplay.setTextColor(SH110X_WHITE);

    oledDisplay.setCursor(25, 0);
    oledDisplay.println("System Boot");

    oledDisplay.setCursor(0, 16);

    // 为避免一行太长显示不下，只保留前 16 个字符。
    char displayText[17];
    strncpy(displayText, statusText, 16);
    displayText[16] = '\0';
    oledDisplay.print("> ");
    oledDisplay.println(displayText);

    // 绘制进度条外框。
    oledDisplay.drawRect(barX, barY, barWidth, barHeight, SH110X_WHITE);

    // 按百分比填充进度条。
    uint8_t fillWidth = (uint8_t)((progress * (barWidth - 4)) / 100);
    if (fillWidth > 0)
    {
        oledDisplay.fillRect(barX + 2, barY + 2, fillWidth, barHeight - 4, SH110X_WHITE);
    }

    oledDisplay.setCursor(48, 50);
    oledDisplay.print(progress);
    oledDisplay.println("%");

    oledDisplay.display();
}

/**
 * @brief 初始化 OLED 显示屏
 *
 * 业务含义：
 * OLED 是整个系统的“本地可视化窗口”，
 * 所以必须在启动早期就准备好，后续所有初始化状态都显示在这里。
 */
void setupOLED()
{
    // 指定 I2C 引脚与总线速率（400kHz）。
    Wire.begin(SDA_PIN, SCL_PIN, 400000);

    // OLED 初始化失败时进入降级模式，避免系统因单个外设失败卡死。
    if (!oledDisplay.begin(SCREEN_ADDRESS, true))
    {
        g_oledReady = false;
        return;
    }

    g_oledReady = true;
    resetInitModuleStatus();
    setInitModuleStatus(INIT_MODULE_OLED, INIT_OK, "Ready");
    showBootProgress(0, "OLED Ready");
}

bool isOLEDReady()
{
    return g_oledReady;
}
