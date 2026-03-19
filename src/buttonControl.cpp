#include "buttonControl.h"

// ===== 回调函数指针 =====
static ButtonCallback g_btn1Cb = nullptr;
static ButtonCallback g_btn2Cb = nullptr;

// ===== 中断共享变量（volatile 保证编译器不优化掉对其的读写）=====
// ISR 只写这两个字段，pollButtons() 负责读取并消抖。
static volatile bool g_btn1Pending = false;
static volatile uint32_t g_btn1IsrMs = 0;

static volatile bool g_btn2Pending = false;
static volatile uint32_t g_btn2IsrMs = 0;

// ===== 冷却时间戳（防止松开时弹起抖动被误判为新按键）=====
static uint32_t g_btn1LastFireMs = 0;
static uint32_t g_btn2LastFireMs = 0;

// ===== 中断服务程序（IRAM_ATTR：存放在 IRAM，保证执行速度）=====
// 只做最轻量的操作：记录时间戳 + 置位标志，不调用任何库函数。

/**
 * @brief 按键 1 中断服务程序（下降沿触发）
 *
 * 说明：仅记录触发时间戳并置位 pending 标志，
 * 消抖和回调调用由 pollButtons() 在主循环中处理。
 */
static void IRAM_ATTR btn1ISR()
{
    g_btn1IsrMs = millis(); // ESP32 的 millis() 在 ISR 中安全可用
    g_btn1Pending = true;
}

/**
 * @brief 按键 2 中断服务程序（下降沿触发）
 *
 * 说明：仅记录触发时间戳并置位 pending 标志，
 * 消抖和回调调用由 pollButtons() 在主循环中处理。
 */
static void IRAM_ATTR btn2ISR()
{
    g_btn2IsrMs = millis();
    g_btn2Pending = true;
}

/**
 * @brief 初始化按键引脚并绑定中断
 *
 * 说明：
 * 1. 将 BTN1_PIN、BTN2_PIN 配置为内部上拉输入。
 * 2. 绑定下降沿中断（按键按下 → GPIO 由 HIGH 变 LOW）。
 * 3. 使用中断 + 主循环消抖的混合方案，既响应灵敏又避免抖动误触。
 */
void setupButtons()
{
    pinMode(BTN1_PIN, INPUT_PULLUP);
    pinMode(BTN2_PIN, INPUT_PULLUP);

    // 绑定下降沿中断（按键按下 → GPIO 由 HIGH 变 LOW）
    attachInterrupt(digitalPinToInterrupt(BTN1_PIN), btn1ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(BTN2_PIN), btn2ISR, FALLING);

    Serial.printf("[BTN] Initialized (interrupt mode): BTN1=IO%d, BTN2=IO%d\n",
                  BTN1_PIN, BTN2_PIN);
}

/**
 * @brief 注册按键 1 的回调函数
 * @param cb 回调函数指针；传 nullptr 可清除已有回调
 */
void setButton1Callback(ButtonCallback cb) { g_btn1Cb = cb; }

/**
 * @brief 注册按键 2 的回调函数
 * @param cb 回调函数指针；传 nullptr 可清除已有回调
 */
void setButton2Callback(ButtonCallback cb) { g_btn2Cb = cb; }

/**
 * @brief 在主循环中轮询按键状态并执行消抖回调（在 loop() 中持续调用）
 *
 * 说明：
 * 1. 检查 ISR 置位的 pending 标志。
 * 2. 距 ISR 触发时间超过 BTN_DEBOUNCE_MS 后，确认为有效按下并执行回调。
 * 3. 本函数无阻塞，不影响主循环帧率。
 */
void pollButtons()
{
    uint32_t now = millis();

    // ----- 按键1 -----
    // 中断已在按下瞬间设置 g_btn1Pending；
    // 这里等消抖时间到期后确认，再执行回调并清除标志。
    if (g_btn1Pending && (now - g_btn1IsrMs) >= BTN_DEBOUNCE_MS)
    {
        g_btn1Pending = false;
        // 冷却期内忽略（防止松开时弹起抖动触发二次回调）
        if ((now - g_btn1LastFireMs) >= BTN_COOLDOWN_MS)
        {
            g_btn1LastFireMs = now;
            if (g_btn1Cb != nullptr)
            {
                g_btn1Cb();
            }
        }
    }

    // ----- 按键2 -----
    if (g_btn2Pending && (now - g_btn2IsrMs) >= BTN_DEBOUNCE_MS)
    {
        g_btn2Pending = false;
        if ((now - g_btn2LastFireMs) >= BTN_COOLDOWN_MS)
        {
            g_btn2LastFireMs = now;
            if (g_btn2Cb != nullptr)
            {
                g_btn2Cb();
            }
        }
    }
}
