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
    const int32_t raw1 = static_cast<int32_t>(getWeight());
    const int32_t raw2 = static_cast<int32_t>(getWeight2());
    const int32_t raw3 = static_cast<int32_t>(getWeight3());

    // 判断各路是否超出传感器标称量程上限（5 kg）
    const bool over1 = (raw1 > HX711_MAX_RANGE_G);
    const bool over2 = (raw2 > HX711_MAX_RANGE_G);
    const bool over3 = (raw3 > HX711_MAX_RANGE_G);

    if (over1 || over2 || over3)
    {
        Serial.printf("[WEIGHT] 超量程: B1=%ldg%s B2=%ldg%s B3=%ldg%s\n",
                      (long)raw1, over1 ? "(OVER)" : "",
                      (long)raw2, over2 ? "(OVER)" : "",
                      (long)raw3, over3 ? "(OVER)" : "");
    }

    // 超量程路钳位到量程上限，使满载告警逻辑（weight >= g_fullWeightG）仍然成立；
    // 负数读数归零，属于传感器噪声或去皮误差，不视为异常
    g_currentWeight1 = over1 ? HX711_MAX_RANGE_G : (raw1 < 0 ? 0 : raw1);
    g_currentWeight2 = over2 ? HX711_MAX_RANGE_G : (raw2 < 0 ? 0 : raw2);
    g_currentWeight3 = over3 ? HX711_MAX_RANGE_G : (raw3 < 0 ? 0 : raw3);

    // 同步超量程状态到 OLED，让重量页显示 OVER 而非普通克数
    setWeightOverRange(over1, over2, over3);

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
