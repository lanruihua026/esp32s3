#include "hx711_2.h"

#include <math.h>

// ===== 第二个 HX711 引脚定义 =====
#define HX711_DT2 11
#define HX711_SCK2 12

// ===== 称重参数 =====
static float calibration_factor2 = 1000.0f;
static float zero_offset2 = 0.0f;

static bool isScaleReady2 = false;

static float s_probeRaw2  = 0.0f;
static int   s_probeValid2 = 0;

static int8_t scale_direction2 = 1;
static bool direction_locked2 = false;

static const float ZERO_DEADBAND_G2 = 2.0f;
static const float DIRECTION_LOCK_RAW_THRESHOLD2 = 30000.0f;

static int32_t readRawData2();
static float readAverageRaw2(int samples, int *outValidCount = nullptr);

static bool waitDataReady2(uint32_t timeout_us)
{
    uint32_t start = micros();
    while (digitalRead(HX711_DT2) == HIGH)
    {
        if (micros() - start > timeout_us)
        {
            return false;
        }
        yield();
    }
    return true;
}

bool setupHX711_2()
{
    pinMode(HX711_DT2, INPUT);
    pinMode(HX711_SCK2, OUTPUT);
    digitalWrite(HX711_SCK2, LOW);
    delay(1200);
    for (int i = 0; i < 8; ++i)
    {
        readRawData2();
    }

    float probeRaw = readAverageRaw2(5, &s_probeValid2);
    s_probeRaw2 = probeRaw;
    if (probeRaw == 0.0f)
    {
        isScaleReady2 = false;
        Serial.println("HX711_2: init failed (probe timeout)");
        return false;
    }

    isScaleReady2 = true;
    tareScale2();
    return true;
}

static int32_t readRawData2()
{
    if (!waitDataReady2(100000))
    {
        Serial.println("HX711_2: Data not ready!");
        return 0;
    }

    uint32_t data = 0;

    for (int i = 23; i >= 0; i--)
    {
        digitalWrite(HX711_SCK2, HIGH);
        delayMicroseconds(1);

        if (digitalRead(HX711_DT2) == HIGH)
        {
            data |= (1U << i);
        }

        digitalWrite(HX711_SCK2, LOW);
        delayMicroseconds(1);
    }

    // 第 25 个时钟脉冲：保持 A 通道、增益 128
    digitalWrite(HX711_SCK2, HIGH);
    delayMicroseconds(1);
    digitalWrite(HX711_SCK2, LOW);
    delayMicroseconds(1);

    if (data & 0x800000)
    {
        data |= 0xFF000000;
    }

    return (int32_t)data;
}

static float readAverageRaw2(int samples, int *outValidCount)
{
    if (samples <= 0)
    {
        if (outValidCount != nullptr) *outValidCount = 0;
        return 0.0f;
    }

    int64_t sum = 0;
    int validCount = 0;

    for (int i = 0; i < samples; i++)
    {
        int32_t raw = readRawData2();
        if (raw != 0)
        {
            sum += raw;
            validCount++;
        }
    }

    if (outValidCount != nullptr) *outValidCount = validCount;

    if (validCount == 0)
    {
        return 0.0f;
    }

    return (float)sum / validCount;
}

float getWeight2()
{
    if (!isScaleReady2)
    {
        return 0.0f;
    }

    const int samples = 5;
    int validCount = 0;
    float rawValue = readAverageRaw2(samples, &validCount);
    if (validCount == 0) return 0.0f;
    float netValue = rawValue - zero_offset2;

    if (!direction_locked2 && fabsf(netValue) > DIRECTION_LOCK_RAW_THRESHOLD2)
    {
        scale_direction2 = (netValue >= 0.0f) ? 1 : -1;
        direction_locked2 = true;
        Serial.print("HX711_2 direction locked: ");
        Serial.println(scale_direction2);
    }

    float weight = (netValue * scale_direction2) / calibration_factor2;

    if (fabsf(weight) < ZERO_DEADBAND_G2)
    {
        weight = 0.0f;
    }

    if (weight < 0)
    {
        weight = 0;
    }

    return weight;
}

void calibrateScale2(float knownWeight)
{
    if (!isScaleReady2)
    {
        Serial.println("Scale2 not ready!");
        return;
    }

    Serial.print("Calibrating HX711_2 with known weight: ");
    Serial.print(knownWeight);
    Serial.println("g");

    const int samples = 10;
    float currentValue = readAverageRaw2(samples);
    float netValue = currentValue - zero_offset2;

    if (knownWeight > 0 && netValue != 0.0f)
    {
        calibration_factor2 = netValue / knownWeight;
    }

    Serial.print("HX711_2 new calibration factor: ");
    Serial.println(calibration_factor2);
}

void tareScale2()
{
    if (!isScaleReady2)
    {
        return;
    }

    const int samples = 15;
    zero_offset2 = readAverageRaw2(samples);

    direction_locked2 = false;
}

void setCalibrationFactor2(float factor)
{
    if (factor != 0.0f)
    {
        calibration_factor2 = factor;
    }
}

void hx711GetProbeResult2(float *raw, int *validCount)
{
    if (raw)        *raw        = s_probeRaw2;
    if (validCount) *validCount = s_probeValid2;
}
