#include "servoControl.h"

// 全局舵机对象：
// 负责驱动垃圾桶分拣机构转到指定角度。
Servo servo;

void initServo()
{
    // 将舵机挂接到指定引脚，并先回到默认位置。
    servo.attach(SERVO_PIN);
    setServoAngle(0);
}

void setServoAngle(int angle)
{
    // 对角度做边界保护，避免传入非法值导致舵机动作异常。
    if (angle < 0)
    {
        angle = 0;
    }
    else if (angle > 180)
    {
        angle = 180;
    }

    servo.write(angle);
}

void runServoSelfTest()
{
    // 开机自检动作：
    // 让舵机从默认位置转到中间角度，再回到原位，
    // 便于观察供电、驱动和机械结构是否正常。
    const int testAngle = 90;
    const uint16_t settleMs = 450;

    setServoAngle(0);
    delay(settleMs);
    setServoAngle(testAngle);
    delay(settleMs);
    setServoAngle(0);
    delay(settleMs);
}
