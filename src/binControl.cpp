#include "binControl.h"

#include "appState.h"
#include "buzzer.h"
#include "hx711.h"
#include "hx711_2.h"
#include "hx711_3.h"
#include "oledInit.h"
#include "onenetMqtt.h"

namespace
{
    float clampPercent(int32_t weight)
    {
        float percent = (weight * 100.0f) / static_cast<float>(g_fullWeightG);
        if (percent < 0.0f)
        {
            return 0.0f;
        }
        if (percent > 100.0f)
        {
            return 100.0f;
        }
        return percent;
    }
}

void updateWeightsAndAlarm()
{
    g_currentWeight1 = static_cast<int32_t>(getWeight());
    g_currentWeight2 = static_cast<int32_t>(getWeight2());
    g_currentWeight3 = static_cast<int32_t>(getWeight3());

    setCurrentWeights(g_currentWeight1, g_currentWeight2, g_currentWeight3);

    const bool anyFull = (g_currentWeight1 >= g_fullWeightG) || (g_currentWeight2 >= g_fullWeightG) || (g_currentWeight3 >= g_fullWeightG);
    const bool allBelow = (g_currentWeight1 < (g_fullWeightG - WARNING_RELEASE_HYSTERESIS_G)) &&
                          (g_currentWeight2 < (g_fullWeightG - WARNING_RELEASE_HYSTERESIS_G)) &&
                          (g_currentWeight3 < (g_fullWeightG - WARNING_RELEASE_HYSTERESIS_G));

    if (!g_warningActive && anyFull)
    {
        g_warningActive = true;
        digitalWrite(WARNING_LIGHT_PIN, HIGH);
        buzzerOn();
    }
    else if (g_warningActive && allBelow)
    {
        g_warningActive = false;
        digitalWrite(WARNING_LIGHT_PIN, LOW);
        buzzerOff();
    }
}

void uploadPropertiesIfNeeded(uint32_t now)
{
    if (!oneNetMqttConnected() || (now - g_lastReportMs) < PROPERTY_REPORT_INTERVAL_MS)
    {
        return;
    }

    g_lastReportMs = now;

    const int32_t nearFullThreshold = g_fullWeightG * 9 / 10;
    const BoxBinData bin1 = {
        g_currentWeight1,
        clampPercent(g_currentWeight1),
        (g_currentWeight1 >= nearFullThreshold) && (g_currentWeight1 < g_fullWeightG),
        g_currentWeight1 >= g_fullWeightG};
    const BoxBinData bin2 = {
        g_currentWeight2,
        clampPercent(g_currentWeight2),
        (g_currentWeight2 >= nearFullThreshold) && (g_currentWeight2 < g_fullWeightG),
        g_currentWeight2 >= g_fullWeightG};
    const BoxBinData bin3 = {
        g_currentWeight3,
        clampPercent(g_currentWeight3),
        (g_currentWeight3 >= nearFullThreshold) && (g_currentWeight3 < g_fullWeightG),
        g_currentWeight3 >= g_fullWeightG};

    oneNetMqttUploadProperties(bin1, bin2, bin3, g_fullWeightG, g_aiConfThreshold);
}

void reportPropertiesNow()
{
    if (!oneNetMqttConnected())
    {
        return;
    }

    const int32_t nearFullThreshold = g_fullWeightG * 9 / 10;
    const BoxBinData bin1 = {
        g_currentWeight1,
        clampPercent(g_currentWeight1),
        (g_currentWeight1 >= nearFullThreshold) && (g_currentWeight1 < g_fullWeightG),
        g_currentWeight1 >= g_fullWeightG};
    const BoxBinData bin2 = {
        g_currentWeight2,
        clampPercent(g_currentWeight2),
        (g_currentWeight2 >= nearFullThreshold) && (g_currentWeight2 < g_fullWeightG),
        g_currentWeight2 >= g_fullWeightG};
    const BoxBinData bin3 = {
        g_currentWeight3,
        clampPercent(g_currentWeight3),
        (g_currentWeight3 >= nearFullThreshold) && (g_currentWeight3 < g_fullWeightG),
        g_currentWeight3 >= g_fullWeightG};

    oneNetMqttUploadProperties(bin1, bin2, bin3, g_fullWeightG, g_aiConfThreshold);
}
