#include "hx711Channel.h"

#include <cmath>
#include <cstdlib>
#include <cstring>

#include "hx711BootConfig.h"

namespace
{
    constexpr float ZERO_DEADBAND_G = 2.0f;
    constexpr float MIN_CALIBRATION_FACTOR_RAW_PER_G = 50.0f;
    constexpr int TARE_MIN_VALID_SAMPLES = 8;
}

Hx711Channel::Hx711Channel(const Hx711ChannelConfig &config) : config_(config)
{
}

bool Hx711Channel::waitDataReady(uint32_t timeoutUs) const
{
    const uint32_t start = micros();
    while (digitalRead(config_.dtPin) == HIGH)
    {
        if (micros() - start > timeoutUs)
        {
            return false;
        }
        yield();
    }
    return true;
}

int32_t Hx711Channel::readRawData() const
{
    if (!waitDataReady(100000))
    {
        return 0;
    }

    uint32_t data = 0;
    for (int i = 23; i >= 0; --i)
    {
        digitalWrite(config_.sckPin, HIGH);
        delayMicroseconds(1);

        if (digitalRead(config_.dtPin) == HIGH)
        {
            data |= (1U << i);
        }

        digitalWrite(config_.sckPin, LOW);
        delayMicroseconds(1);
    }

    digitalWrite(config_.sckPin, HIGH);
    delayMicroseconds(1);
    digitalWrite(config_.sckPin, LOW);
    delayMicroseconds(1);

    if (data & 0x800000)
    {
        data |= 0xFF000000;
    }
    return static_cast<int32_t>(data);
}

float Hx711Channel::readAverageRaw(int samples, int *outValidCount) const
{
    if (samples <= 0)
    {
        if (outValidCount != nullptr)
        {
            *outValidCount = 0;
        }
        return 0.0f;
    }

    int64_t sum = 0;
    int validCount = 0;
    for (int i = 0; i < samples; ++i)
    {
        const int32_t raw = readRawData();
        if (raw != 0)
        {
            sum += raw;
            ++validCount;
        }
    }

    if (outValidCount != nullptr)
    {
        *outValidCount = validCount;
    }
    return validCount > 0 ? static_cast<float>(sum) / validCount : 0.0f;
}

void Hx711Channel::setScaleDirection(int8_t direction)
{
    if (direction > 0)
    {
        scaleDirection_ = 1;
    }
    else if (direction < 0)
    {
        scaleDirection_ = -1;
    }
    else
    {
        scaleDirection_ = 0;
    }
}

bool Hx711Channel::setup()
{
    pinMode(config_.dtPin, INPUT);
    pinMode(config_.sckPin, OUTPUT);
    digitalWrite(config_.sckPin, LOW);

    if (HX711_BOOT_STRATEGY == 0)
    {
        delay(HX711_PER_CHANNEL_WARMUP_MS);
    }

    for (int i = 0; i < 8; ++i)
    {
        readRawData();
    }

    hardwareReady_ = readAverageRaw(5) != 0.0f;
    return hardwareReady_;
}

bool Hx711Channel::init(Preferences &prefs, bool prefsOk, float defaultScale, float bootEmptyThresholdG)
{
    if (!setup())
    {
        return false;
    }

    if (!prefsOk)
    {
        setCalibrationFactor(defaultScale);
        zeroValid_ = tare();
        return zeroValid_;
    }

    if (prefs.isKey(config_.scaleKey))
    {
        setCalibrationFactor(prefs.getFloat(config_.scaleKey, defaultScale));
    }
    else
    {
        setCalibrationFactor(defaultScale);
        prefs.putFloat(config_.scaleKey, defaultScale);
    }

    if (prefs.isKey(config_.directionKey))
    {
        setScaleDirection(static_cast<int8_t>(prefs.getChar(config_.directionKey, 0)));
    }
    else
    {
        setScaleDirection(0);
    }

    if (HX711_BOOT_AUTO_TARE)
    {
        zeroValid_ = tare();
        if (zeroValid_)
        {
            prefs.putFloat(config_.zeroKey, getZeroOffset());
            Serial.printf("%s boot auto tare zero=%.0f\r\n", config_.label, getZeroOffset());
            return true;
        }
        Serial.printf("%s boot auto tare failed; fallback to stored zero\r\n", config_.label);
    }

    if (prefs.isKey(config_.zeroKey))
    {
        setZeroOffset(prefs.getFloat(config_.zeroKey, 0.0f));
        zeroValid_ = true;

        const float loadMagnitude = getLoadMagnitude();
        const float rezeroThreshold = fmaxf(bootEmptyThresholdG, config_.bootRezeroThresholdG);
        if (loadMagnitude >= 0.0f && loadMagnitude < rezeroThreshold && tare())
        {
            prefs.putFloat(config_.zeroKey, getZeroOffset());
        }
        else if (loadMagnitude >= rezeroThreshold)
        {
            Serial.printf("%s boot zero differs by %.1fg; empty bin then send %s or TAREALL\r\n",
                          config_.label,
                          loadMagnitude,
                          config_.tareCommand);
        }
        return true;
    }

    zeroValid_ = tare();
    if (zeroValid_)
    {
        prefs.putFloat(config_.zeroKey, getZeroOffset());
    }
    return zeroValid_;
}

float Hx711Channel::getWeight()
{
    if (!hardwareReady_)
    {
        return 0.0f;
    }

    int validCount = 0;
    const float rawValue = readAverageRaw(5, &validCount);
    if (validCount == 0)
    {
        return 0.0f;
    }

    const float netValue = rawValue - zeroOffset_;
    float weight = (scaleDirection_ == 0) ? (fabsf(netValue) / calibrationFactor_) : ((netValue * scaleDirection_) / calibrationFactor_);
    if (fabsf(weight) < ZERO_DEADBAND_G)
    {
        weight = 0.0f;
    }
    return weight < 0.0f ? 0.0f : weight;
}

float Hx711Channel::getLoadMagnitude()
{
    if (!hardwareReady_)
    {
        return -1.0f;
    }

    int validCount = 0;
    const float rawValue = readAverageRaw(5, &validCount);
    if (validCount == 0)
    {
        return -1.0f;
    }

    return fabsf(rawValue - zeroOffset_) / calibrationFactor_;
}

bool Hx711Channel::calibrate(float knownWeight)
{
    if (!hardwareReady_ || knownWeight <= 0.0f)
    {
        return false;
    }

    int validCount = 0;
    const float currentValue = readAverageRaw(10, &validCount);
    if (validCount < 5)
    {
        return false;
    }

    const float netValue = currentValue - zeroOffset_;
    if (netValue == 0.0f)
    {
        return false;
    }

    const float newCalibrationFactor = fabsf(netValue) / knownWeight;
    if (newCalibrationFactor < MIN_CALIBRATION_FACTOR_RAW_PER_G)
    {
        return false;
    }

    calibrationFactor_ = newCalibrationFactor;
    setScaleDirection((netValue >= 0.0f) ? 1 : -1);
    return true;
}

bool Hx711Channel::tare()
{
    if (!hardwareReady_)
    {
        return false;
    }

    int validCount = 0;
    const float avg = readAverageRaw(15, &validCount);
    if (validCount < TARE_MIN_VALID_SAMPLES)
    {
        return false;
    }

    zeroOffset_ = avg;
    zeroValid_ = true;
    return true;
}

float Hx711Channel::getZeroOffset() const
{
    return zeroOffset_;
}

void Hx711Channel::setZeroOffset(float offset)
{
    zeroOffset_ = offset;
    zeroValid_ = true;
}

void Hx711Channel::setCalibrationFactor(float factor)
{
    if (factor != 0.0f)
    {
        calibrationFactor_ = fabsf(factor);
    }
}

float Hx711Channel::getCalibrationFactor() const
{
    return calibrationFactor_;
}

bool Hx711Channel::handleCommand(const char *cmd, Preferences &prefs, bool prefsOk)
{
    if (cmd == nullptr || cmd[0] == '\0')
    {
        return false;
    }

    if (strcmp(cmd, config_.tareCommand) == 0)
    {
        calibrationReadyForCommand_ = false;
        if (tare())
        {
            if (prefsOk)
            {
                prefs.putFloat(config_.zeroKey, getZeroOffset());
            }
            calibrationReadyForCommand_ = true;
            Serial.printf("%s TARE OK zero=%.0f\r\n", config_.label, getZeroOffset());
        }
        else
        {
            Serial.printf("%s TARE FAIL: HX711 not ready or too few samples\r\n", config_.label);
        }
        return true;
    }

    const size_t prefixLen = strlen(config_.calCommandPrefix);
    if (strncmp(cmd, config_.calCommandPrefix, prefixLen) == 0)
    {
        const float knownWeight = static_cast<float>(atof(cmd + prefixLen));
        if (knownWeight > 0.0f && calibrationReadyForCommand_ && calibrate(knownWeight))
        {
            if (prefsOk)
            {
                prefs.putFloat(config_.scaleKey, getCalibrationFactor());
                prefs.putChar(config_.directionKey, scaleDirection_);
            }
            Serial.printf("%s CAL OK scale=%.3f dir=%d\r\n", config_.label, getCalibrationFactor(), static_cast<int>(scaleDirection_));
            calibrationReadyForCommand_ = false;
        }
        else
        {
            Serial.printf("%s CAL FAIL: run %s on empty bin, then %s<known_g>; check raw response if this repeats\r\n",
                          config_.label,
                          config_.tareCommand,
                          config_.calCommandPrefix);
        }
        return true;
    }

    return false;
}

void Hx711Channel::clearStoredCalibration(Preferences &prefs, bool prefsOk)
{
    setScaleDirection(0);
    calibrationReadyForCommand_ = false;
    if (prefsOk)
    {
        prefs.remove(config_.scaleKey);
        prefs.remove(config_.zeroKey);
        prefs.remove(config_.directionKey);
    }
    Serial.printf("%s stored HX711 calibration cleared; reboot or send %s on empty bin\r\n",
                  config_.label,
                  config_.tareCommand);
}

void Hx711Channel::printStatus(Print &out, int32_t reportedWeight)
{
    int validCount = 0;
    const float rawValue = readAverageRaw(5, &validCount);
    const float netValue = rawValue - zeroOffset_;
    const float calculatedWeight = (validCount == 0 || !hardwareReady_)
                                       ? 0.0f
                                       : ((scaleDirection_ == 0) ? (fabsf(netValue) / calibrationFactor_) : ((netValue * scaleDirection_) / calibrationFactor_));
    const char *direction = (scaleDirection_ > 0) ? "+" : ((scaleDirection_ < 0) ? "-" : "auto");

    out.printf("%s ready=%d valid=%d raw=%.0f zero=%.0f net=%.0f scale=%.3f dir=%s calc=%.1fg report=%ldg\r\n",
               config_.label,
               hardwareReady_ ? 1 : 0,
               validCount,
               rawValue,
               zeroOffset_,
               netValue,
               calibrationFactor_,
               direction,
               calculatedWeight,
               static_cast<long>(reportedWeight));
}

void Hx711Channel::printRawSamples(Print &out, uint8_t samples, uint16_t delayMs)
{
    if (samples == 0)
    {
        samples = 1;
    }
    if (samples > 30)
    {
        samples = 30;
    }

    out.printf("%s RAW samples=%u zero=%.0f scale=%.3f dir=%s\r\n",
               config_.label,
               static_cast<unsigned>(samples),
               zeroOffset_,
               calibrationFactor_,
               (scaleDirection_ > 0) ? "+" : ((scaleDirection_ < 0) ? "-" : "auto"));

    for (uint8_t i = 0; i < samples; ++i)
    {
        const int32_t raw = readRawData();
        const float netValue = static_cast<float>(raw) - zeroOffset_;
        const float grams = fabsf(netValue) / calibrationFactor_;
        out.printf("%s RAW[%02u] raw=%ld net=%.0f abs=%.1fg\r\n",
                   config_.label,
                   static_cast<unsigned>(i + 1),
                   static_cast<long>(raw),
                   netValue,
                   grams);

        if (delayMs > 0 && i + 1 < samples)
        {
            delay(delayMs);
        }
    }
}
