#ifndef _BUTTON_CONTROL_H_
#define _BUTTON_CONTROL_H_

#include <Arduino.h>

// 两个实体按键当前只用于切换 OLED 页面。
#define BTN1_PIN 9
#define BTN2_PIN 10

// 按键采用“中断捕获 + 主循环消抖”的方案。
#define BTN_DEBOUNCE_MS 20
#define BTN_COOLDOWN_MS 300

// 按键稳定触发后执行的业务回调。
typedef void (*ButtonCallback)();

// 初始化按键引脚和中断。
void setupButtons();

// 注册按键 1/2 的业务回调。
void setButton1Callback(ButtonCallback cb);
void setButton2Callback(ButtonCallback cb);

// 在 loop() 中持续调用，用于做消抖和回调分发。
void pollButtons();

#endif
