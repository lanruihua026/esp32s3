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

static int clampAngle(int angle)
{
    if (angle < 0) return 0;
    if (angle > 180) return 180;
    return angle;
}

void setServoAngle(int angle)
{
    servo.write(clampAngle(angle));
}

void setServoAngle2(int angle)
{
    servo2.write(clampAngle(angle));
}

void setServoAngle3(int angle)
{
    servo3.write(clampAngle(angle));
}

void runServoSelfTest()
{
    // 开机自检动作：三路舵机依次从 0° 转到 90° 再回到 0°，
    // 便于观察供电、驱动和机械结构是否正常。
    const int testAngle = 90;
    const uint16_t settleMs = 450;

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
