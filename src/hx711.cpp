#include "hx711.h"

#include <cstring>

#include "hx711Channel.h"
#include "hx711_2.h"
#include "hx711_3.h"

namespace
{
    Hx711Channel g_channel1({
        1,
        2,
        "hx1_scale",
        "hx1_zero",
        "TARE1",
        "CAL1:",
        150.0f,
    });

    bool handleHx711Command1(const char *cmd, Preferences &prefs, bool prefsOk)
    {
        return g_channel1.handleCommand(cmd, prefs, prefsOk);
    }
}

bool setupHX711()
{
    return g_channel1.setup();
}

float getWeight()
{
    return g_channel1.getWeight();
}

float getLoadMagnitude()
{
    return g_channel1.getLoadMagnitude();
}

bool calibrateScale(float knownWeight)
{
    return g_channel1.calibrate(knownWeight);
}

bool tareScale()
{
    return g_channel1.tare();
}

void setCalibrationFactor(float factor)
{
    g_channel1.setCalibrationFactor(factor);
}

float getCalibrationFactor()
{
    return g_channel1.getCalibrationFactor();
}

float getZeroOffset()
{
    return g_channel1.getZeroOffset();
}

void setZeroOffset(float offset)
{
    g_channel1.setZeroOffset(offset);
}

bool initHx711Channel1(Preferences &prefs, bool prefsOk, float defaultScale, float bootEmptyThreshold)
{
    return g_channel1.init(prefs, prefsOk, defaultScale, bootEmptyThreshold);
}

void pollHx711SerialCommands(Preferences &prefs, bool prefsOk, int32_t, int32_t, int32_t)
{
    static char s_serialCmdBuf[64] = {0};
    static uint8_t s_serialCmdPos = 0;

    while (Serial.available())
    {
        const char c = static_cast<char>(Serial.read());
        if (c == '\r')
        {
            continue;
        }
        if (c == '\n')
        {
            s_serialCmdBuf[s_serialCmdPos] = '\0';
            for (uint8_t i = 0; i < s_serialCmdPos; ++i)
            {
                if (s_serialCmdBuf[i] >= 'a' && s_serialCmdBuf[i] <= 'z')
                {
                    s_serialCmdBuf[i] -= 32;
                }
            }

            if (s_serialCmdPos > 0)
            {
                const bool handled = handleHx711Command1(s_serialCmdBuf, prefs, prefsOk) ||
                                     handleHx711Command2(s_serialCmdBuf, prefs, prefsOk, 0) ||
                                     handleHx711Command3(s_serialCmdBuf, prefs, prefsOk, 0);
                (void)handled;
            }
            s_serialCmdPos = 0;
            continue;
        }

        if (s_serialCmdPos < sizeof(s_serialCmdBuf) - 1)
        {
            s_serialCmdBuf[s_serialCmdPos++] = c;
        }
        else
        {
            s_serialCmdPos = 0;
        }
    }
}
