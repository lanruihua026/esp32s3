#include "buzzer.h"
/**
 * @brief 初始化蜂鸣器
 * 触发方式：低电平触发
 */
void buzzerInit()
{
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, HIGH); // 初始状态关闭蜂鸣器
}

/**
 * @brief 蜂鸣器发声
 * @param duration_ms 发声持续时间（毫秒）
 */
void buzzerBeep(uint32_t duration_ms)
{
    digitalWrite(BUZZER_PIN, LOW);  // 打开蜂鸣器
    delay(duration_ms);             // 持续指定时间
    digitalWrite(BUZZER_PIN, HIGH); // 关闭蜂鸣器
}
