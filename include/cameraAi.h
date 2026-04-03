#ifndef CAMERA_AI_H
#define CAMERA_AI_H

#include <Arduino.h>

void expireAiStateIfStale(uint32_t now);
void pollCameraUart();
void updateServoByAiResult();

#endif
