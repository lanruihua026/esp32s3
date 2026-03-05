#include "hx711.h"

#include <math.h>

// ===== HX711 引脚定义（请按实际接线修改） =====
#define HX711_DT 1
#define HX711_SCK 2

// ===== 称重参数 =====
// calibration_factor 含义：每 1 克对应的原始 ADC 差值（raw / g）
static float calibration_factor = 1000.0f;
// zero_offset 含义：去皮后的零点偏移（原始 ADC）
static float zero_offset = 0.0f;

// 初始化完成标志
static bool isScaleReady = false;

// 部分接线或受力方向下，施加重量可能使原始值变小（负方向）。
// scale_direction 用于统一把“加重”映射为正重量。
static int8_t scale_direction = 1;
// 一旦检测到足够明显的受力变化，就锁定方向，避免来回抖动。
static bool direction_locked = false;

// 小重量死区：将非常小的噪声视为 0g
static const float ZERO_DEADBAND_G = 2.0f;
// 方向锁定阈值：只有净原始值足够大才判断方向
static const float DIRECTION_LOCK_RAW_THRESHOLD = 30000.0f;

static int32_t readRawData();
static float readAverageRaw(int samples);

/**
 * @brief 等待 HX711 数据就绪（DT 由高变低）
 * @param timeout_us 超时时间（微秒）
 * @return true 数据就绪；false 超时
 */
static bool waitDataReady(uint32_t timeout_us)
{
    uint32_t start = micros();
    while (digitalRead(HX711_DT) == HIGH)
    {
        if (micros() - start > timeout_us)
        {
            return false;
        }
    }
    return true;
}

void setupHX711()
{
    Serial.println("Initializing HX711...");

    pinMode(HX711_DT, INPUT);
    pinMode(HX711_SCK, OUTPUT);
    digitalWrite(HX711_SCK, LOW);

    // 上电后给传感器一点预热时间，降低初始漂移影响
    delay(1200);

    // 丢弃若干帧启动阶段的不稳定数据
    for (int i = 0; i < 8; ++i)
    {
        readRawData();
    }

    isScaleReady = true;
    tareScale();
    Serial.println("HX711 initialized. Place known weight for calibration.");
}

/**
 * @brief 读取一帧 24 位原始值（增益 128，通道 A）
 * @return 原始值（带符号 int32），失败返回 0
 */
static int32_t readRawData()
{
    // 最长等待 100ms，避免阻塞过久
    if (!waitDataReady(100000))
    {
        Serial.println("HX711: Data not ready!");
        return 0;
    }

    uint32_t data = 0;

    // HX711 24 位数据，高位先出
    for (int i = 23; i >= 0; i--)
    {
        digitalWrite(HX711_SCK, HIGH);
        delayMicroseconds(1);

        if (digitalRead(HX711_DT) == HIGH)
        {
            data |= (1U << i);
        }

        digitalWrite(HX711_SCK, LOW);
        delayMicroseconds(1);
    }

    // 第 25 个时钟脉冲用于设置下一次转换的通道/增益。
    // 这里保持 A 通道、增益 128。
    digitalWrite(HX711_SCK, HIGH);
    delayMicroseconds(1);
    digitalWrite(HX711_SCK, LOW);
    delayMicroseconds(1);

    // 将 24 位补码扩展为 32 位有符号整数
    if (data & 0x800000)
    {
        data |= 0xFF000000;
    }

    return (int32_t)data;
}

/**
 * @brief 多次采样求均值
 *
 * 说明：readRawData() 失败会返回 0，这里把失败样本剔除。
 */
static float readAverageRaw(int samples)
{
    if (samples <= 0)
    {
        return 0.0f;
    }

    int64_t sum = 0;
    int validCount = 0;

    for (int i = 0; i < samples; i++)
    {
        int32_t raw = readRawData();
        if (raw != 0)
        {
            sum += raw;
            validCount++;
        }
    }

    if (validCount == 0)
    {
        return 0.0f;
    }

    return (float)sum / validCount;
}

float getWeight()
{
    if (!isScaleReady)
    {
        return 0.0f;
    }

    const int samples = 5;
    float rawValue = readAverageRaw(samples);
    float netValue = rawValue - zero_offset;

    // 自动识别受力方向并锁定：
    // 如果受力后净值显著为负，后续统一乘 -1，让重量保持正向增长。
    if (!direction_locked && fabsf(netValue) > DIRECTION_LOCK_RAW_THRESHOLD)
    {
        scale_direction = (netValue >= 0.0f) ? 1 : -1;
        direction_locked = true;
        Serial.print("HX711 direction locked: ");
        Serial.println(scale_direction);
    }

    float weight = (netValue * scale_direction) / calibration_factor;

    // 小抖动直接压到 0，提升静止显示稳定性
    if (fabsf(weight) < ZERO_DEADBAND_G)
    {
        weight = 0.0f;
    }

    // 业务上不允许负重量
    if (weight < 0)
    {
        weight = 0;
    }

    return weight;
}

void calibrateScale(float knownWeight)
{
    if (!isScaleReady)
    {
        Serial.println("Scale not ready!");
        return;
    }

    Serial.print("Calibrating with known weight: ");
    Serial.print(knownWeight);
    Serial.println("g");

    const int samples = 10;
    float currentValue = readAverageRaw(samples);
    float netValue = currentValue - zero_offset;

    // calibration_factor = 原始净值 / 实际重量
    if (knownWeight > 0 && netValue != 0.0f)
    {
        calibration_factor = netValue / knownWeight;
    }

    Serial.print("New calibration factor: ");
    Serial.println(calibration_factor);
}

void tareScale()
{
    if (!isScaleReady)
    {
        return;
    }

    Serial.println("Taring scale...");

    // 读取当前空载平均值作为零点偏移
    const int samples = 15;
    zero_offset = readAverageRaw(samples);

    // 重新去皮后允许再次识别方向
    direction_locked = false;

    Serial.print("Zero offset: ");
    Serial.println(zero_offset);
}

void setCalibrationFactor(float factor)
{
    if (factor != 0.0f)
    {
        calibration_factor = factor;
    }
}
