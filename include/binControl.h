#ifndef BIN_CONTROL_H
#define BIN_CONTROL_H

#include <Arduino.h>

void updateWeightsAndAlarm();
void uploadPropertiesIfNeeded(uint32_t now);
void reportPropertiesNow();

#endif
