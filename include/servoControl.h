#ifndef __SERVO_CONTROL_H__
#define __SERVO_CONTROL_H__

#include <Arduino.h>
#include <ESP32Servo.h>

// ===== 舵机 GPIO 引脚定义 =====
#define SERVO_PIN   7  // 舵机 1（分拣机构第一路）信号线所在 GPIO 引脚
#define SERVO_PIN_2 8  // 舵机 2（分拣机构第二路）信号线所在 GPIO 引脚
#define SERVO_PIN_3 16 // 舵机 3（分拣机构第三路）信号线所在 GPIO 引脚

/**
 * @brief 初始化三路舵机，绑定引脚并回到默认位置（0°）
 */
void initServo();

/**
 * @brief 设置第一路舵机角度
 * @param angle 目标角度（度，0~180，超限自动截断）
 */
void setServoAngle(int angle);

/**
 * @brief 设置第二路舵机角度
 * @param angle 目标角度（度，0~180，超限自动截断）
 */
void setServoAngle2(int angle);

/**
 * @brief 设置第三路舵机角度
 * @param angle 目标角度（度，0~180，超限自动截断）
 */
void setServoAngle3(int angle);

/**
 * @brief 舵机开机自检：三路舵机各转至 90° 后回到 0°
 *
 * 在系统上电初始化阶段调用，用于验证舵机供电、驱动与机械结构是否正常。
 */
void runServoSelfTest();

#endif
