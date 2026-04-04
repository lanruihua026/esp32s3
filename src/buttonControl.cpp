#include "buttonControl.h"
#include "oledInit.h"

// 各按键最终触发的业务回调。
static ButtonCallback g_btn1Cb = nullptr;
static ButtonCallback g_btn2Cb = nullptr;
static ButtonCallback g_btn3Cb = nullptr;

// ISR 只记录“发生过按下”这件事，真正的消抖和业务处理放到 loop() 中完成。
static volatile bool g_btn1Pending = false;
static volatile uint32_t g_btn1IsrMs = 0;
static volatile bool g_btn2Pending = false;
static volatile uint32_t g_btn2IsrMs = 0;
static volatile bool g_btn3Pending = false;
static volatile uint32_t g_btn3IsrMs = 0;

// 冷却时间用于过滤松手回弹带来的二次触发。
static uint32_t g_btn1LastFireMs = 0;
static uint32_t g_btn2LastFireMs = 0;
static uint32_t g_btn3LastFireMs = 0;

// 按键 1 中断：只做最轻量的记录。
static void IRAM_ATTR btn1ISR()
{
    g_btn1IsrMs = millis();
    g_btn1Pending = true;
}

// 按键 2 中断：只做最轻量的记录。
static void IRAM_ATTR btn2ISR()
{
    g_btn2IsrMs = millis();
    g_btn2Pending = true;
}

// 按键 3 中断：只做最轻量的记录。
static void IRAM_ATTR btn3ISR()
{
    g_btn3IsrMs = millis();
    g_btn3Pending = true;
}

void setupButtons()
{
    // 使用内部上拉输入，按下时电平从 HIGH 变为 LOW。
    pinMode(BTN1_PIN, INPUT_PULLUP);
    pinMode(BTN2_PIN, INPUT_PULLUP);
    pinMode(BTN3_PIN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(BTN1_PIN), btn1ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(BTN2_PIN), btn2ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(BTN3_PIN), btn3ISR, FALLING);
}

void initButtonsForOledNavigation()
{
    setupButtons();
    setButton1Callback(prevOledPage);
    setButton2Callback(toggleOledPage);
}

void setButton1Callback(ButtonCallback cb)
{
    g_btn1Cb = cb;
}

void setButton2Callback(ButtonCallback cb)
{
    g_btn2Cb = cb;
}

void setButton3Callback(ButtonCallback cb)
{
    g_btn3Cb = cb;
}

void pollButtons()
{
    const uint32_t now = millis();

    // 按键 1：等待消抖时间到，再决定是否触发业务回调。
    if (g_btn1Pending && (now - g_btn1IsrMs) >= BTN_DEBOUNCE_MS)
    {
        g_btn1Pending = false;
        if ((now - g_btn1LastFireMs) >= BTN_COOLDOWN_MS)
        {
            g_btn1LastFireMs = now;
            if (g_btn1Cb != nullptr)
            {
                g_btn1Cb();
            }
        }
    }

    // 按键 2：处理流程与按键 1 相同。
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

    // 按键 3：处理流程与按键 1/2 相同。
    if (g_btn3Pending && (now - g_btn3IsrMs) >= BTN_DEBOUNCE_MS)
    {
        g_btn3Pending = false;
        if ((now - g_btn3LastFireMs) >= BTN_COOLDOWN_MS)
        {
            g_btn3LastFireMs = now;
            if (g_btn3Cb != nullptr)
            {
                g_btn3Cb();
            }
        }
    }
}
