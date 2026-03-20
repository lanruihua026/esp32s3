#ifndef _BUZZER_H_
#define _BUZZER_H_
#include <Arduino.h>

#define BUZZER_PIN 15 // 有源蜂鸣器 GPIO

/**
 * @brief 初始化蜂鸣器
 * 触发方式：低电平触发
 */
void buzzerInit();
/**
 * @brief 蜂鸣器发声
 * @param duration_ms 发声持续时间（毫秒）
 */
void buzzerBeep(uint32_t duration_ms);

/**
 * @brief 蜂鸣器持续打开
 */
void buzzerOn();
/**
 * @brief 蜂鸣器持续关闭
 */
void buzzerOff();

#endif
