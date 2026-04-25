#include "hx711_3.h"

#include "hx711Channel.h"

namespace
{
    Hx711Channel g_channel3({
        13,
        14,
        "hx3_scale",
        "hx3_zero",
        "TARE3",
        "CAL3:",
        150.0f,
    });
}

bool setupHX711_3()
{
    return g_channel3.setup();
}

float getWeight3()
{
    return g_channel3.getWeight();
}

float getLoadMagnitude3()
{
    return g_channel3.getLoadMagnitude();
}

bool calibrateScale3(float knownWeight)
{
    return g_channel3.calibrate(knownWeight);
}

bool tareScale3()
{
    return g_channel3.tare();
}

void setCalibrationFactor3(float factor)
{
    g_channel3.setCalibrationFactor(factor);
}

float getCalibrationFactor3()
{
    return g_channel3.getCalibrationFactor();
}

float getZeroOffset3()
{
    return g_channel3.getZeroOffset();
}

void setZeroOffset3(float offset)
{
    g_channel3.setZeroOffset(offset);
}

bool initHx711Channel3(Preferences &prefs, bool prefsOk, float defaultScale, float bootEmptyThreshold)
{
    return g_channel3.init(prefs, prefsOk, defaultScale, bootEmptyThreshold);
}

bool handleHx711Command3(const char *cmd, Preferences &prefs, bool prefsOk, int32_t)
{
    return g_channel3.handleCommand(cmd, prefs, prefsOk);
}
