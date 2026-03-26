#include "servoControl.h"

// 全局舵机对象：
// 负责驱动垃圾桶分拣机构转到指定角度。
Servo servo;
Servo servo2;
Servo servo3;

void initServo()
{
    // 将三路舵机挂接到对应引脚，并先回到默认位置。
    servo.attach(SERVO_PIN);
    servo2.attach(SERVO_PIN_2);
    servo3.attach(SERVO_PIN_3);
    setServoAngle(0);
    setServoAngle2(0);
    setServoAngle3(0);
}

/**
 * @brief 将角度限幅到舵机有效范围 [0, 180]
 * @param angle 输入角度（度）
 * @return 限幅后的角度（度）
 *
 * 防止调用方传入超出机械行程的角度，避免堵转损坏舵机。
 */
static int clampAngle(int angle)
{
    if (angle < 0) return 0;
    if (angle > 180) return 180;
    return angle;
}

/**
 * @brief 设置第一路舵机的目标角度
 * @param angle 目标角度（度，有效范围 0~180，超限自动截断）
 */
void setServoAngle(int angle)
{
    servo.write(clampAngle(angle));
}

/**
 * @brief 设置第二路舵机的目标角度
 * @param angle 目标角度（度，有效范围 0~180，超限自动截断）
 */
void setServoAngle2(int angle)
{
    servo2.write(clampAngle(angle));
}

/**
 * @brief 设置第三路舵机的目标角度
 * @param angle 目标角度（度，有效范围 0~180，超限自动截断）
 */
void setServoAngle3(int angle)
{
    servo3.write(clampAngle(angle));
}

void runServoSelfTest()
{
    // 开机自检动作：三路舵机依次从 0° 转到 90° 再回到 0°，
    // 便于观察供电、驱动和机械结构是否正常。
    const int testAngle = 90;       // 自检目标角度（度）
    const uint16_t settleMs = 450;  // 每步动作后的等待时间（毫秒），留给舵机完成转动

    setServoAngle(0);
    setServoAngle2(0);
    setServoAngle3(0);
    delay(settleMs);

    setServoAngle(testAngle);
    setServoAngle2(testAngle);
    setServoAngle3(testAngle);
    delay(settleMs);

    setServoAngle(0);
    setServoAngle2(0);
    setServoAngle3(0);
    delay(settleMs);
}
