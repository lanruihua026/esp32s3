#include "hx711.h"

#include <math.h>

// HX711 传感器引脚定义
// DT (DOUT) 引脚 - 可根据实际接线修改
#define HX711_DT 1
// SCK (CLK) 引脚 - 可根据实际接线修改
#define HX711_SCK 2

// 称重参数（需要根据你的传感器进行校准）
// 参考值：5kg 量程 typical value 约 10000 ~ 50000
// 实际需要校准
static float calibration_factor = 1000.0f;
static float zero_offset = 0.0f;
static bool isScaleReady = false;
static int8_t scale_direction = 1;
static bool direction_locked = false;

static const float ZERO_DEADBAND_G = 2.0f;
static const float DIRECTION_LOCK_RAW_THRESHOLD = 30000.0f;

/**
 * @brief 读取 HX711 原始数据（24位有符号整数）
 * @return 原始ADC值，出错返回0
 */
static int32_t readRawData();

/**
 * @brief 读取多个样本并计算均值（自动丢弃无效样本）
 * @param samples 采样数量
 * @return float 均值；若无有效样本返回0
 */
static float readAverageRaw(int samples);

/**
 * @brief HX711 初始化
 */
void setupHX711()
{
    Serial.println("Initializing HX711...");

    // 设置引脚模式
    pinMode(HX711_DT, INPUT);
    pinMode(HX711_SCK, OUTPUT);

    // 初始化时钟引脚为低电平
    digitalWrite(HX711_SCK, LOW);

    // 预热等待：上电初期 ADC 漂移较大
    delay(1200);

    // 丢弃前几次不稳定数据
    for (int i = 0; i < 8; ++i)
    {
        readRawData();
    }

    isScaleReady = true;
    tareScale();
    Serial.println("HX711 initialized. Place known weight for calibration.");
}

/**
 * @brief 等待数据就绪
 * @return true 数据就绪, false 超时
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

/**
 * @brief 读取 HX711 原始数据（24位有符号整数）
 * @return 原始ADC值，出错返回0
 */
static int32_t readRawData()
{
    // 等待数据就绪（超时100ms）
    if (!waitDataReady(100000))
    {
        Serial.println("HX711: Data not ready!");
        return 0;
    }

    // 临时变量存储24位数据
    uint32_t data = 0;

    // 读取24位数据（高位先出）
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

    // 第25个时钟脉冲，设置增益为128（通道A）
    // 可根据需要修改增益
    digitalWrite(HX711_SCK, HIGH);
    delayMicroseconds(1);
    digitalWrite(HX711_SCK, LOW);
    delayMicroseconds(1);

    // 转换为有符号整数（24位有符号范围：-8388608 ~ 8388607）
    if (data & 0x800000)
    {
        data |= 0xFF000000;
    }

    return (int32_t)data;
}

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

/**
 * @brief 获取当前重量（克）
 * @return float 重量值
 */
float getWeight()
{
    if (!isScaleReady)
    {
        return 0.0f;
    }

    // 读取多次取平均，提高稳定性
    const int samples = 5;
    float rawValue = readAverageRaw(samples);
    float netValue = rawValue - zero_offset;

    // 自动识别称重方向：部分接线下，受力时净值会变负
    if (!direction_locked && fabsf(netValue) > DIRECTION_LOCK_RAW_THRESHOLD)
    {
        scale_direction = (netValue >= 0.0f) ? 1 : -1;
        direction_locked = true;
        Serial.print("HX711 direction locked: ");
        Serial.println(scale_direction);
    }

    // 应用校准因子转换为实际重量
    float weight = (netValue * scale_direction) / calibration_factor;

    if (fabsf(weight) < ZERO_DEADBAND_G)
    {
        weight = 0.0f;
    }

    // 确保重量不为负数
    if (weight < 0)
    {
        weight = 0;
    }

    return weight;
}

/**
 * @brief 校准传感器
 * @param knownWeight 已知的校准重量（克）
 */
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

    // 读取当前平均值
    const int samples = 10;
    float currentValue = readAverageRaw(samples);
    float netValue = currentValue - zero_offset;

    // 计算新校准因子
    // calibration_factor = raw_value / actual_weight
    if (knownWeight > 0 && netValue != 0.0f)
    {
        calibration_factor = netValue / knownWeight;
    }

    Serial.print("New calibration factor: ");
    Serial.println(calibration_factor);
}

/**
 * @brief 重新去皮（归零）
 */
void tareScale()
{
    if (!isScaleReady)
    {
        return;
    }

    Serial.println("Taring scale...");

    // 读取当前平均值作为零点偏移
    const int samples = 15;
    zero_offset = readAverageRaw(samples);
    direction_locked = false;

    Serial.print("Zero offset: ");
    Serial.println(zero_offset);
}

/**
 * @brief 设置校准因子（用于已有校准值时）
 * @param factor 校准因子
 */
void setCalibrationFactor(float factor)
{
    if (factor != 0.0f)
    {
        calibration_factor = factor;
    }
}