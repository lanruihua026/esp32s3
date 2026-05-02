#include "hx711.h"

#include <cstring>

#include "hx711Channel.h"
#include "hx711_2.h"
#include "hx711_3.h"

namespace
{
    Hx711Channel g_channel1({
        "B1",
        1,
        2,
        "hx1_scale",
        "hx1_zero",
        "hx1_dir",
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

void printHx711Status1(Print &out, int32_t currentWeight)
{
    g_channel1.printStatus(out, currentWeight);
}

void printHx711RawSamples1(Print &out, uint8_t samples, uint16_t delayMs)
{
    g_channel1.printRawSamples(out, samples, delayMs);
}

void clearHx711StoredCalibration1(Preferences &prefs, bool prefsOk)
{
    g_channel1.clearStoredCalibration(prefs, prefsOk);
}

void pollHx711SerialCommands(Preferences &prefs, bool prefsOk, int32_t currentWeight1, int32_t currentWeight2, int32_t currentWeight3)
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
                bool handled = false;
                if (strcmp(s_serialCmdBuf, "STATUS") == 0 || strcmp(s_serialCmdBuf, "STATUSALL") == 0)
                {
                    printHx711Status1(Serial, currentWeight1);
                    printHx711Status2(Serial, currentWeight2);
                    printHx711Status3(Serial, currentWeight3);
                    handled = true;
                }
                else if (strcmp(s_serialCmdBuf, "STATUS1") == 0)
                {
                    printHx711Status1(Serial, currentWeight1);
                    handled = true;
                }
                else if (strcmp(s_serialCmdBuf, "STATUS2") == 0)
                {
                    printHx711Status2(Serial, currentWeight2);
                    handled = true;
                }
                else if (strcmp(s_serialCmdBuf, "STATUS3") == 0)
                {
                    printHx711Status3(Serial, currentWeight3);
                    handled = true;
                }
                else if (strcmp(s_serialCmdBuf, "RAWALL") == 0)
                {
                    printHx711RawSamples1(Serial, 10, 80);
                    printHx711RawSamples2(Serial, 10, 80);
                    printHx711RawSamples3(Serial, 10, 80);
                    handled = true;
                }
                else if (strcmp(s_serialCmdBuf, "RAW1") == 0)
                {
                    printHx711RawSamples1(Serial, 15, 80);
                    handled = true;
                }
                else if (strcmp(s_serialCmdBuf, "RAW2") == 0)
                {
                    printHx711RawSamples2(Serial, 15, 80);
                    handled = true;
                }
                else if (strcmp(s_serialCmdBuf, "RAW3") == 0)
                {
                    printHx711RawSamples3(Serial, 15, 80);
                    handled = true;
                }
                else if (strcmp(s_serialCmdBuf, "TAREALL") == 0)
                {
                    handled = true;
                    (void)handleHx711Command1("TARE1", prefs, prefsOk);
                    (void)handleHx711Command2("TARE2", prefs, prefsOk, currentWeight2);
                    (void)handleHx711Command3("TARE3", prefs, prefsOk, currentWeight3);
                }
                else if (strcmp(s_serialCmdBuf, "CLEARHX") == 0 || strcmp(s_serialCmdBuf, "CLEARHXALL") == 0)
                {
                    clearHx711StoredCalibration1(prefs, prefsOk);
                    clearHx711StoredCalibration2(prefs, prefsOk);
                    clearHx711StoredCalibration3(prefs, prefsOk);
                    handled = true;
                }
                else if (strcmp(s_serialCmdBuf, "CLEARHX1") == 0)
                {
                    clearHx711StoredCalibration1(prefs, prefsOk);
                    handled = true;
                }
                else if (strcmp(s_serialCmdBuf, "CLEARHX2") == 0)
                {
                    clearHx711StoredCalibration2(prefs, prefsOk);
                    handled = true;
                }
                else if (strcmp(s_serialCmdBuf, "CLEARHX3") == 0)
                {
                    clearHx711StoredCalibration3(prefs, prefsOk);
                    handled = true;
                }
                else
                {
                    handled = handleHx711Command1(s_serialCmdBuf, prefs, prefsOk) ||
                              handleHx711Command2(s_serialCmdBuf, prefs, prefsOk, currentWeight2) ||
                              handleHx711Command3(s_serialCmdBuf, prefs, prefsOk, currentWeight3);
                }
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
