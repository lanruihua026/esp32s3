#include "hx711_3.h"

#include <math.h>

// ===== 第三个 HX711 引脚定义 =====
#define HX711_DT3 13
#define HX711_SCK3 14

// ===== 称重参数 =====
static float calibration_factor3 = 1000.0f;
static float zero_offset3 = 0.0f;

static bool isScaleReady3 = false;

static float s_probeRaw3  = 0.0f;
static int   s_probeValid3 = 0;

static int8_t scale_direction3 = 1;
static bool direction_locked3 = false;

static const float ZERO_DEADBAND_G3 = 2.0f;
static const float DIRECTION_LOCK_RAW_THRESHOLD3 = 30000.0f;

static int32_t readRawData3();
static float readAverageRaw3(int samples, int *outValidCount = nullptr);

static bool waitDataReady3(uint32_t timeout_us)
{
    uint32_t start = micros();
    while (digitalRead(HX711_DT3) == HIGH)
    {
        if (micros() - start > timeout_us)
        {
            return false;
        }
        yield();
    }
    return true;
}

bool setupHX711_3()
{
    pinMode(HX711_DT3, INPUT);
    pinMode(HX711_SCK3, OUTPUT);
    digitalWrite(HX711_SCK3, LOW);
    delay(1200);
    for (int i = 0; i < 8; ++i)
    {
        readRawData3();
    }

    float probeRaw = readAverageRaw3(5, &s_probeValid3);
    s_probeRaw3 = probeRaw;
    if (probeRaw == 0.0f)
    {
        isScaleReady3 = false;
        Serial.println("HX711_3: init failed (probe timeout)");
        return false;
    }

    isScaleReady3 = true;
    tareScale3();
    return true;
}

static int32_t readRawData3()
{
    if (!waitDataReady3(100000))
    {
        Serial.println("HX711_3: Data not ready!");
        return 0;
    }

    uint32_t data = 0;

    for (int i = 23; i >= 0; i--)
    {
        digitalWrite(HX711_SCK3, HIGH);
        delayMicroseconds(1);

        if (digitalRead(HX711_DT3) == HIGH)
        {
            data |= (1U << i);
        }

        digitalWrite(HX711_SCK3, LOW);
        delayMicroseconds(1);
    }

    // 第 25 个时钟脉冲：保持 A 通道、增益 128
    digitalWrite(HX711_SCK3, HIGH);
    delayMicroseconds(1);
    digitalWrite(HX711_SCK3, LOW);
    delayMicroseconds(1);

    if (data & 0x800000)
    {
        data |= 0xFF000000;
    }

    return (int32_t)data;
}

static float readAverageRaw3(int samples, int *outValidCount)
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
        int32_t raw = readRawData3();
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

float getWeight3()
{
    if (!isScaleReady3)
    {
        return 0.0f;
    }

    const int samples = 5;
    int validCount = 0;
    float rawValue = readAverageRaw3(samples, &validCount);
    if (validCount == 0) return 0.0f;
    float netValue = rawValue - zero_offset3;

    if (!direction_locked3 && fabsf(netValue) > DIRECTION_LOCK_RAW_THRESHOLD3)
    {
        scale_direction3 = (netValue >= 0.0f) ? 1 : -1;
        direction_locked3 = true;
        Serial.print("HX711_3 direction locked: ");
        Serial.println(scale_direction3);
    }

    float weight = (netValue * scale_direction3) / calibration_factor3;

    if (fabsf(weight) < ZERO_DEADBAND_G3)
    {
        weight = 0.0f;
    }

    if (weight < 0)
    {
        weight = 0;
    }

    return weight;
}

void calibrateScale3(float knownWeight)
{
    if (!isScaleReady3)
    {
        Serial.println("Scale3 not ready!");
        return;
    }

    Serial.print("Calibrating HX711_3 with known weight: ");
    Serial.print(knownWeight);
    Serial.println("g");

    const int samples = 10;
    float currentValue = readAverageRaw3(samples);
    float netValue = currentValue - zero_offset3;

    if (knownWeight > 0 && netValue != 0.0f)
    {
        calibration_factor3 = netValue / knownWeight;
    }

    Serial.print("HX711_3 new calibration factor: ");
    Serial.println(calibration_factor3);
}

void tareScale3()
{
    if (!isScaleReady3)
    {
        return;
    }

    const int samples = 15;
    zero_offset3 = readAverageRaw3(samples);

    direction_locked3 = false;
}

void setCalibrationFactor3(float factor)
{
    if (factor != 0.0f)
    {
        calibration_factor3 = factor;
    }
}

void hx711GetProbeResult3(float *raw, int *validCount)
{
    if (raw)        *raw        = s_probeRaw3;
    if (validCount) *validCount = s_probeValid3;
}
