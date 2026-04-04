#ifndef BIN_CONTROL_H
#define BIN_CONTROL_H

#include <Arduino.h>

void updateWeightsAndAlarm();
/** 满溢时按解除键：关闭警示灯与蜂鸣器；重量与 OLED 显示逻辑不变。脱离满载后可再次触发声光。 */
void silenceOverflowAlarmBuzzer();
void uploadPropertiesIfNeeded(uint32_t now);
void reportPropertiesNow();

#endif
