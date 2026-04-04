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
    // 用户按「解除警告」后，在仍满载（anyFull）期间抑制声光；脱离满载（!anyFull）或 allBelow 时清除。
    static bool g_overflowSoundSuppressed = false;

    /**
     * @brief 将重量转换为百分比，并限制在0%到100%之间
     * @param weight 当前重量，单位克
     * @return 返回对应的百分比值，0.0f表示0%，100.0f表示100%，如果重量超过满载重量则返回100%，如果重量为负则返回0%
     */
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

/**
 * @brief 更新当前重量读数并根据满载重量阈值控制警示灯和蜂鸣器
 */
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

    if (allBelow)
    {
        g_overflowSoundSuppressed = false;
        g_warningActive = false;
        digitalWrite(WARNING_LIGHT_PIN, LOW);
        buzzerOff();
    }
    else if (anyFull)
    {
        if (g_overflowSoundSuppressed)
        {
            g_warningActive = false;
            digitalWrite(WARNING_LIGHT_PIN, LOW);
            buzzerOff();
        }
        else if (!g_warningActive)
        {
            g_warningActive = true;
            digitalWrite(WARNING_LIGHT_PIN, HIGH);
            buzzerOn();
        }
    }
    else
    {
        // 灰区：!anyFull && !allBelow；未按解除键时保持原「直到 allBelow 才自动关声光」行为（不写 GPIO）。
        if (g_overflowSoundSuppressed)
        {
            g_warningActive = false;
            digitalWrite(WARNING_LIGHT_PIN, LOW);
            buzzerOff();
            g_overflowSoundSuppressed = false;
        }
    }
}

void silenceOverflowAlarmBuzzer()
{
    g_overflowSoundSuppressed = true;
    g_warningActive = false;
    digitalWrite(WARNING_LIGHT_PIN, LOW);
    buzzerOff();
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
