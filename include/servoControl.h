#ifndef __SERVO_CONTROL_H__
#define __SERVO_CONTROL_H__

#include <Arduino.h>

// 舵机 GPIO 引脚定义。
#define SERVO_PIN   7
#define SERVO_PIN_2 8
#define SERVO_PIN_3 16

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
 * 编译选项 -DSERVO_FULL_SELFTEST_AT_BOOT=0 可跳过本自检；initServo() 仍会回零。
 *
 * @param beforeChannel 每路自检开始前回调；channel 为 1~3。若关闭完整自检，则调用一次 channel==0（仅回零模式）。
 */
void runServoSelfTest(void (*beforeChannel)(uint8_t channel) = nullptr);

#endif
