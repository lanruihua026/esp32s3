#ifndef _BUTTON_CONTROL_H_
#define _BUTTON_CONTROL_H_

#include <Arduino.h>

// ===== 按键引脚定义 =====
#define BTN1_PIN 9  // 按键1：IO9，按下点亮 RGB LED
#define BTN2_PIN 10 // 按键2：IO10，按下关闭 RGB LED

// ===== 消抖参数 =====
#define BTN_DEBOUNCE_MS 20  // 消抖时间（ms）：等待抖动稳定
#define BTN_COOLDOWN_MS 300 // 冷却时间（ms）：有效触发后屏蔽松开时的弹起抖动

/**
 * @brief 按键回调函数类型
 *
 * 在按键稳定触发（下降沿）时被调用一次。
 */
typedef void (*ButtonCallback)();

/**
 * @brief 初始化两个按键 GPIO（上拉输入）
 */
void setupButtons();

/**
 * @brief 注册按键1按下时的回调函数
 * @param cb 回调函数指针
 */
void setButton1Callback(ButtonCallback cb);

/**
 * @brief 注册按键2按下时的回调函数
 * @param cb 回调函数指针
 */
void setButton2Callback(ButtonCallback cb);

/**
 * @brief 按键轮询（应在 loop() 中持续调用）
 *
 * 中断负责瞬时捕获按键边沿并设置标志，
 * 本函数只做消抖确认与回调分发，全程非阻塞。
 */
void pollButtons();

#endif // _BUTTON_CONTROL_H_
