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

// ===== 中断服务程序（IRAM_ATTR：存放在 IRAM，保证执行速度）=====
// 只做最轻量的操作：记录时间戳 + 置位标志，不调用任何库函数。
static void IRAM_ATTR btn1ISR()
{
    g_btn1IsrMs = millis(); // ESP32 的 millis() 在 ISR 中安全可用
    g_btn1Pending = true;
}

static void IRAM_ATTR btn2ISR()
{
    g_btn2IsrMs = millis();
    g_btn2Pending = true;
}

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

void setButton1Callback(ButtonCallback cb) { g_btn1Cb = cb; }
void setButton2Callback(ButtonCallback cb) { g_btn2Cb = cb; }

void pollButtons()
{
    uint32_t now = millis();

    // ----- 按键1 -----
    // 中断已在按下瞬间设置 g_btn1Pending；
    // 这里等消抖时间到期后确认，再执行回调并清除标志。
    if (g_btn1Pending && (now - g_btn1IsrMs) >= BTN_DEBOUNCE_MS)
    {
        g_btn1Pending = false;
        if (g_btn1Cb != nullptr)
        {
            g_btn1Cb();
        }
    }

    // ----- 按键2 -----
    if (g_btn2Pending && (now - g_btn2IsrMs) >= BTN_DEBOUNCE_MS)
    {
        g_btn2Pending = false;
        if (g_btn2Cb != nullptr)
        {
            g_btn2Cb();
        }
    }
}
