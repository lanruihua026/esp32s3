#ifndef __SERVO_CONTROL_H__
#define __SERVO_CONTROL_H__

#include <Arduino.h>

// ===== 舵机 GPIO 引脚定义 =====
#define SERVO_PIN   7  // 舵机 1（电池仓分拣机构）信号线所在 GPIO 引脚
#define SERVO_PIN_2 8  // 舵机 2（手机仓分拣机构）信号线所在 GPIO 引脚
#define SERVO_PIN_3 16 // 舵机 3（数码配件仓分拣机构）信号线所在 GPIO 引脚

/**
 * @brief 初始化三路舵机
 *
 * 使用 ESP32-S3 原生 LEDC 外设为三路舵机分别生成独立的 50Hz PWM。
 * 这样每个舵机都占用固定通道，不再依赖第三方库的自动分配逻辑，
 * 可避免不同舵机被错误映射到同一底层 PWM 信号而一起动作。
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
 * 在系统上电初始化阶段调用，用于验证：
 * 1. LEDC 通道是否初始化成功；
 * 2. 舵机供电和共地是否正常；
 * 3. 机械结构是否存在卡滞。
 *
 * 自检采用“逐路动作”的方式，便于现场直接观察是哪一路不转或串动。
 */
void runServoSelfTest();

#endif
