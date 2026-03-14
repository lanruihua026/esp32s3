#include "servoControl.h"

Servo servo; // 创建舵机对象

/**
 * @brief 初始化舵机
 * 说明：该函数在 setup() 中被调用，负责连接舵机到指定 GPIO 引脚，并进行必要的配置。
 * 注意：确保 SERVO_PIN 定义的引脚支持 PWM 输出，并且连接正确。
 */
void initServo()
{
    servo.attach(SERVO_PIN); // 连接舵机到指定引脚
}

/**
 * @brief 设置舵机角度
 * @param angle 角度值（0-180）
 */

void setServoAngle(int angle)
{
    servo.write(angle); // 设置舵机角度
}