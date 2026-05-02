#ifndef HX711_CHANNEL_H
#define HX711_CHANNEL_H

#include <Arduino.h>
#include <Preferences.h>

struct Hx711ChannelConfig
{
    const char *label;
    uint8_t dtPin;
    uint8_t sckPin;
    const char *scaleKey;
    const char *zeroKey;
    const char *directionKey;
    const char *tareCommand;
    const char *calCommandPrefix;
    float bootRezeroThresholdG;
};

class Hx711Channel
{
public:
    explicit Hx711Channel(const Hx711ChannelConfig &config);

    bool setup();
    bool init(Preferences &prefs, bool prefsOk, float defaultScale, float bootEmptyThresholdG);
    float getWeight();
    float getLoadMagnitude();
    bool calibrate(float knownWeight);
    bool tare();
    float getZeroOffset() const;
    void setZeroOffset(float offset);
    void setCalibrationFactor(float factor);
    float getCalibrationFactor() const;
    bool handleCommand(const char *cmd, Preferences &prefs, bool prefsOk);
    void clearStoredCalibration(Preferences &prefs, bool prefsOk);
    void printStatus(Print &out, int32_t reportedWeight);
    void printRawSamples(Print &out, uint8_t samples, uint16_t delayMs);

private:
    bool waitDataReady(uint32_t timeoutUs) const;
    int32_t readRawData() const;
    float readAverageRaw(int samples, int *outValidCount = nullptr) const;
    void setScaleDirection(int8_t direction);

    Hx711ChannelConfig config_;
    float calibrationFactor_ = 1000.0f;
    float zeroOffset_ = 0.0f;
    int8_t scaleDirection_ = 0;
    bool hardwareReady_ = false;
    bool zeroValid_ = false;
    bool calibrationReadyForCommand_ = false;
};

#endif
