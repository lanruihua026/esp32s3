#include "servoControl.h"

// 编译可加 -DSERVO_FULL_SELFTEST_AT_BOOT=0 跳过完整自检，仅保留 initServo() 内回零（加快启动）
#ifndef SERVO_FULL_SELFTEST_AT_BOOT
#define SERVO_FULL_SELFTEST_AT_BOOT 1
#endif

namespace
{
    /**
     * 本文件改用 ESP32-S3 原生 LEDC 外设驱动舵机。
     *
     * 设计目标：
     * 1. 每个舵机固定占用一个独立 LEDC 通道，彻底绕开 ESP32Servo 在 S3 上的 MCPWM 映射问题。
     * 2. 所有时间参数、脉宽参数集中定义，便于后续根据实际舵机型号微调。
     * 3. 提供明确的初始化、自检和写入失败状态，方便后续按需添加诊断。
     */
    constexpr uint32_t SERVO_PWM_FREQ_HZ = 50;             // 标准舵机刷新频率：50Hz，即周期 20ms
    constexpr uint8_t SERVO_PWM_RES_BITS = 14;             // ESP32-S3 LEDC 在低频下使用 14 位分辨率，精度足够且兼容性好
    constexpr uint32_t SERVO_PWM_MAX_DUTY = (1UL << SERVO_PWM_RES_BITS) - 1UL;
    constexpr uint32_t SERVO_PERIOD_US = 1000000UL / SERVO_PWM_FREQ_HZ; // 20,000us
    constexpr uint16_t SERVO_MIN_PULSE_US = 500;          // 0° 典型控制脉宽
    constexpr uint16_t SERVO_MAX_PULSE_US = 2400;         // 180° 典型控制脉宽
    constexpr int SERVO_SELFTEST_ANGLE = 90;              // 开机自检转到中位，便于观察
    constexpr uint16_t SERVO_SETTLE_MS = 600;             // 每一步动作后的等待时间，保证转动足够明显

    // 为三路舵机显式指定不同的 LEDC 通道，确保它们的 PWM 信号完全独立。
    constexpr uint8_t SERVO1_CHANNEL = 0;
    constexpr uint8_t SERVO2_CHANNEL = 1;
    constexpr uint8_t SERVO3_CHANNEL = 2;

    struct ServoRuntime
    {
        const char *name;  // 舵机名称
        uint8_t pin;       // 输出引脚
        uint8_t channel;   // 固定绑定的 LEDC 通道
        bool ready;        // 该路 LEDC 是否初始化成功
        int lastAngle;     // 最近一次写入的角度，便于日志和后续扩展
    };

    ServoRuntime g_servo1 = {"servo1", SERVO_PIN, SERVO1_CHANNEL, false, 0};
    ServoRuntime g_servo2 = {"servo2", SERVO_PIN_2, SERVO2_CHANNEL, false, 0};
    ServoRuntime g_servo3 = {"servo3", SERVO_PIN_3, SERVO3_CHANNEL, false, 0};

    /**
     * @brief 将输入角度限制在舵机安全范围 [0,180]
     */
    int clampAngle(int angle)
    {
        if (angle < 0)
        {
            return 0;
        }
        if (angle > 180)
        {
            return 180;
        }
        return angle;
    }

    /**
     * @brief 将角度转换为舵机需要的高电平脉宽（单位：微秒）
     *
     * 采用线性映射：
     * 0°   -> 500us
     * 180° -> 2400us
     */
    uint32_t angleToPulseUs(int angle)
    {
        const int safeAngle = clampAngle(angle);
        const uint32_t spanUs = static_cast<uint32_t>(SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);
        return SERVO_MIN_PULSE_US + (spanUs * static_cast<uint32_t>(safeAngle)) / 180UL;
    }

    /**
     * @brief 将脉宽转换为 LEDC 占空比计数值
     *
     * LEDC 的 duty 取值范围是 [0, 2^resolution - 1]。
     * 对 50Hz 舵机来说，周期固定为 20ms，因此 duty = pulse_us / 20000us * max_duty。
     */
    uint32_t pulseUsToDuty(uint32_t pulseUs)
    {
        return (pulseUs * SERVO_PWM_MAX_DUTY) / SERVO_PERIOD_US;
    }

    /**
     * @brief 初始化单路舵机的 LEDC 输出
     *
     * 当前工程使用的 Arduino-ESP32 版本仍是旧版 LEDC API，
     * 因此这里采用两步式初始化：
     * 1. ledcSetup(channel, freq, bits) 配置指定通道；
     * 2. ledcAttachPin(pin, channel) 把引脚绑定到该通道。
     *
     * 这样仍然能够保证“三路舵机各占一个固定通道”，避免自动分配带来的串动问题。
     */
    bool initSingleServo(ServoRuntime &servo)
    {
        const double actualFreq = ledcSetup(servo.channel, SERVO_PWM_FREQ_HZ, SERVO_PWM_RES_BITS);
        const bool setupOk = (actualFreq > 0.0);

        if (!setupOk)
        {
            return false;
        }

        ledcAttachPin(servo.pin, servo.channel);

        servo.ready = true;
        servo.lastAngle = 0;
        return true;
    }

    /**
     * @brief 向单路舵机写入角度
     *
     * 这里不直接写“角度”，而是显式算出脉宽和 duty 后再调用 ledcWrite(channel, duty)，
     * 这样波形完全可控，便于排查问题。
     */
    void writeServoAngle(ServoRuntime &servo, int angle)
    {
        if (!servo.ready)
        {
            return;
        }

        const int safeAngle = clampAngle(angle);
        const uint32_t pulseUs = angleToPulseUs(safeAngle);
        const uint32_t duty = pulseUsToDuty(pulseUs);
        ledcWrite(servo.channel, duty);

        servo.lastAngle = safeAngle;
    }

    /**
     * @brief 单路舵机自检
     *
     * 顺序为 0° -> 90° -> 0°，逐路执行。
     * 逐路动作比三路同时动作更容易看出哪一路异常，也更适合现场调试。
     */
    void runSingleServoSelfTest(ServoRuntime &servo, uint16_t settleMs)
    {
        if (!servo.ready)
        {
            return;
        }

        writeServoAngle(servo, 0);
        delay(settleMs);

        writeServoAngle(servo, SERVO_SELFTEST_ANGLE);
        delay(settleMs);

        writeServoAngle(servo, 0);
        delay(settleMs);
    }
} // namespace

void initServo()
{
    // 三路舵机分别绑定到固定 LEDC 通道，避免任何自动通道分配和后端切换。
    g_servo1.ready = initSingleServo(g_servo1);
    g_servo2.ready = initSingleServo(g_servo2);
    g_servo3.ready = initSingleServo(g_servo3);

    // 初始化成功后统一回到 0°，保证分拣机构从已知位置开始工作。
    setServoAngle(0);
    setServoAngle2(0);
    setServoAngle3(0);
}

void setServoAngle(int angle)
{
    writeServoAngle(g_servo1, angle);
}

void setServoAngle2(int angle)
{
    writeServoAngle(g_servo2, angle);
}

void setServoAngle3(int angle)
{
    writeServoAngle(g_servo3, angle);
}

void runServoSelfTest(void (*beforeChannel)(uint8_t channel))
{
#if SERVO_FULL_SELFTEST_AT_BOOT
    constexpr uint16_t kSettle = SERVO_SETTLE_MS;
    if (beforeChannel)
    {
        beforeChannel(1);
    }
    runSingleServoSelfTest(g_servo1, kSettle);
    if (beforeChannel)
    {
        beforeChannel(2);
    }
    runSingleServoSelfTest(g_servo2, kSettle);
    if (beforeChannel)
    {
        beforeChannel(3);
    }
    runSingleServoSelfTest(g_servo3, kSettle);
#else
    if (beforeChannel)
    {
        beforeChannel(0);
    }
#endif
}
