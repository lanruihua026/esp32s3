#include "hx711Channel.h"

#include <cmath>
#include <cstdlib>
#include <cstring>

#include "hx711BootConfig.h"

namespace
{
    constexpr float ZERO_DEADBAND_G = 2.0f;
    constexpr float DIRECTION_LOCK_RAW_THRESHOLD = 30000.0f;
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

void Hx711Channel::resetDirection()
{
    scaleDirection_ = 1;
    directionLocked_ = false;
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
    int8_t appliedDirection = scaleDirection_;
    if (!directionLocked_ && fabsf(netValue) > DIRECTION_LOCK_RAW_THRESHOLD)
    {
        scaleDirection_ = (netValue >= 0.0f) ? 1 : -1;
        directionLocked_ = true;
        appliedDirection = scaleDirection_;
    }
    else if (!directionLocked_)
    {
        appliedDirection = (netValue >= 0.0f) ? 1 : -1;
    }

    float weight = (netValue * appliedDirection) / calibrationFactor_;
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

    calibrationFactor_ = fabsf(netValue) / knownWeight;
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
    resetDirection();
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
    resetDirection();
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
            }
            calibrationReadyForCommand_ = false;
        }
        return true;
    }

    return strcmp(cmd, "STATUS") == 0;
}
