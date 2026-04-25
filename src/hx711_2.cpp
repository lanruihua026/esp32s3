#include "hx711_2.h"

#include "hx711Channel.h"

namespace
{
    Hx711Channel g_channel2({
        11,
        12,
        "hx2_scale",
        "hx2_zero",
        "TARE2",
        "CAL2:",
        150.0f,
    });
}

bool setupHX711_2()
{
    return g_channel2.setup();
}

float getWeight2()
{
    return g_channel2.getWeight();
}

float getLoadMagnitude2()
{
    return g_channel2.getLoadMagnitude();
}

bool calibrateScale2(float knownWeight)
{
    return g_channel2.calibrate(knownWeight);
}

bool tareScale2()
{
    return g_channel2.tare();
}

void setCalibrationFactor2(float factor)
{
    g_channel2.setCalibrationFactor(factor);
}

float getCalibrationFactor2()
{
    return g_channel2.getCalibrationFactor();
}

float getZeroOffset2()
{
    return g_channel2.getZeroOffset();
}

void setZeroOffset2(float offset)
{
    g_channel2.setZeroOffset(offset);
}

bool initHx711Channel2(Preferences &prefs, bool prefsOk, float defaultScale, float bootEmptyThreshold)
{
    return g_channel2.init(prefs, prefsOk, defaultScale, bootEmptyThreshold);
}

bool handleHx711Command2(const char *cmd, Preferences &prefs, bool prefsOk, int32_t)
{
    return g_channel2.handleCommand(cmd, prefs, prefsOk);
}
