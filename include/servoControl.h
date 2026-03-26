#ifndef __SERVO_CONTROL_H__
#define __SERVO_CONTROL_H__

#include <Arduino.h>
#include <ESP32Servo.h>

#define SERVO_PIN   7  // 舵机 1 GPIO 引脚
#define SERVO_PIN_2 8  // 舵机 2 GPIO 引脚
#define SERVO_PIN_3 16 // 舵机 3 GPIO 引脚

void initServo();
void setServoAngle(int angle);
void setServoAngle2(int angle);
void setServoAngle3(int angle);
void runServoSelfTest();

#endif
