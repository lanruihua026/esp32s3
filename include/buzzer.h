#ifndef _BUZZER_H_
#define _BUZZER_H_

#include <Arduino.h>

// 本项目使用有源蜂鸣器，低电平响，高电平停。
#define BUZZER_PIN 15

// 初始化蜂鸣器到默认静音状态。
void buzzerInit();

// 蜂鸣器短响一段时间，适合提示音。
void buzzerBeep(uint32_t duration_ms);

// 持续打开/关闭蜂鸣器，适合超重告警。
void buzzerOn();
void buzzerOff();

#endif
