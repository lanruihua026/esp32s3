#ifndef __SERVO_CONTROL_H__
#define __SERVO_CONTROL_H__

#include <Arduino.h>
#include <ESP32Servo.h>

#define SERVO_PIN 7 // 连接舵机的 GPIO 引脚

void initServo();
void setServoAngle(int angle);
void runServoSelfTest();

#endif
