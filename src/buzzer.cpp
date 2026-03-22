#include "buzzer.h"

void buzzerInit()
{
    // 上电先配置为输出，并保持静音，防止误响。
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, HIGH);
}

void buzzerBeep(uint32_t duration_ms)
{
    // 用于需要“响一下”的场景，比如人工提示。
    digitalWrite(BUZZER_PIN, LOW);
    delay(duration_ms);
    digitalWrite(BUZZER_PIN, HIGH);
}

void buzzerOn()
{
    // 低电平表示持续鸣叫。
    digitalWrite(BUZZER_PIN, LOW);
}

void buzzerOff()
{
    // 高电平表示关闭蜂鸣器。
    digitalWrite(BUZZER_PIN, HIGH);
}
