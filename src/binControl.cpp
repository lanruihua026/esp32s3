#include "binControl.h" // 仓体控制接口
#include "appState.h"   // 全局状态
#include "buzzer.h"     // 蜂鸣器控制
#include "hx711.h"      // 1号称重模块
#include "hx711_2.h"    // 2号称重模块
#include "hx711_3.h"    // 3号称重模块
#include "oledInit.h"   // OLED 显示
#include "onenetMqtt.h" // OneNET 上报

namespace // 匿名命名空间
{         // 开始
    // 用户按「解除警告」后，在仍满载（anyFull）期间抑制声光；脱离满载（!anyFull）或 allBelow 时清除。
    static bool g_overflowSoundSuppressed = false; // 解除后的抑制标志

    /**
     * @brief 将重量转换为百分比，并限制在0%到100%之间
     * @param weight 当前重量，单位克
     * @return 返回对应的百分比值，0.0f表示0%，100.0f表示100%，如果重量超过满载重量则返回100%，如果重量为负则返回0%
     */
    float clampPercent(int32_t weight)                                         // 计算并限幅百分比
    {                                                                          // 函数开始
        float percent = (weight * 100.0f) / static_cast<float>(g_fullWeightG); // 计算占比
        if (percent < 0.0f)                                                    // 低于下限
        {                                                                      // 分支开始
            return 0.0f;                                                       // 返回0%
        } // 分支结束
        if (percent > 100.0f) // 高于上限
        {                     // 分支开始
            return 100.0f;    // 返回100%
        } // 分支结束
        return percent; // 返回结果
    } // 函数结束
} // 匿名命名空间结束

/**
 * @brief 更新当前重量读数并根据满载重量阈值控制警示灯和蜂鸣器
 */
void updateWeightsAndAlarm()                                 // 更新重量并处理告警
{                                                            // 函数开始
    const int32_t raw1 = static_cast<int32_t>(getWeight());  // 读取1号重量
    const int32_t raw2 = static_cast<int32_t>(getWeight2()); // 读取2号重量
    const int32_t raw3 = static_cast<int32_t>(getWeight3()); // 读取3号重量

    // 判断各路是否超出传感器标称量程上限（5 kg）
    const bool over1 = (raw1 > HX711_MAX_RANGE_G); // 1号是否超量程
    const bool over2 = (raw2 > HX711_MAX_RANGE_G); // 2号是否超量程
    const bool over3 = (raw3 > HX711_MAX_RANGE_G); // 3号是否超量程

    // 超量程路钳位到量程上限，使满载告警逻辑（weight >= g_fullWeightG）仍然成立；
    // 负数读数归零，属于传感器噪声或去皮误差，不视为异常
    g_currentWeight1 = over1 ? HX711_MAX_RANGE_G : (raw1 < 0 ? 0 : raw1); // 1号重量限幅
    g_currentWeight2 = over2 ? HX711_MAX_RANGE_G : (raw2 < 0 ? 0 : raw2); // 2号重量限幅
    g_currentWeight3 = over3 ? HX711_MAX_RANGE_G : (raw3 < 0 ? 0 : raw3); // 3号重量限幅

    // 同步超量程状态到 OLED，让重量页显示 OVER 而非普通克数
    setWeightOverRange(over1, over2, over3); // 更新显示状态

    setCurrentWeights(g_currentWeight1, g_currentWeight2, g_currentWeight3); // 同步当前重量

    const bool anyFull = (g_currentWeight1 >= g_fullWeightG) || (g_currentWeight2 >= g_fullWeightG) || (g_currentWeight3 >= g_fullWeightG); // 任一路满载
    const bool allBelow = (g_currentWeight1 < (g_fullWeightG - WARNING_RELEASE_HYSTERESIS_G)) &&                                            // 1号低于释放阈值
                          (g_currentWeight2 < (g_fullWeightG - WARNING_RELEASE_HYSTERESIS_G)) &&                                            // 2号低于释放阈值
                          (g_currentWeight3 < (g_fullWeightG - WARNING_RELEASE_HYSTERESIS_G));                                              // 3号低于释放阈值

    if (allBelow)                             // 全部回到安全区
    {                                         // 分支开始
        g_overflowSoundSuppressed = false;    // 清除抑制标志
        g_warningActive = false;              // 关闭告警状态
        digitalWrite(WARNING_LIGHT_PIN, LOW); // 关闭警示灯
        buzzerOff();                          // 关闭蜂鸣器
    } // 分支结束
    else if (anyFull)                             // 任一路进入满载区
    {                                             // 分支开始
        if (g_overflowSoundSuppressed)            // 已按解除键
        {                                         // 分支开始
            g_warningActive = false;              // 不触发告警
            digitalWrite(WARNING_LIGHT_PIN, LOW); // 保持警示灯关闭
            buzzerOff();                          // 保持蜂鸣器关闭
        } // 分支结束
        else if (!g_warningActive)                 // 首次进入告警
        {                                          // 分支开始
            g_warningActive = true;                // 标记告警已激活
            digitalWrite(WARNING_LIGHT_PIN, HIGH); // 点亮警示灯
            buzzerOn();                            // 打开蜂鸣器
        } // 分支结束
    } // 分支结束
    else // 灰区处理
    {    // 分支开始
        // 灰区：!anyFull && !allBelow；未按解除键时保持原「直到 allBelow 才自动关声光」行为（不写 GPIO）。
        if (g_overflowSoundSuppressed)            // 已解除且离开满载
        {                                         // 分支开始
            g_warningActive = false;              // 清除告警状态
            digitalWrite(WARNING_LIGHT_PIN, LOW); // 关闭警示灯
            buzzerOff();                          // 关闭蜂鸣器
            g_overflowSoundSuppressed = false;    // 恢复自动告警
        } // 分支结束
    } // 分支结束
} // 函数结束

void silenceOverflowAlarmBuzzer()         // 手动静音告警
{                                         // 函数开始
    g_overflowSoundSuppressed = true;     // 置为已解除
    g_warningActive = false;              // 清除告警状态
    digitalWrite(WARNING_LIGHT_PIN, LOW); // 关闭警示灯
    buzzerOff();                          // 关闭蜂鸣器
} // 函数结束

void uploadPropertiesIfNeeded(uint32_t now)                                             // 按周期上报属性
{                                                                                       // 函数开始
    if (!oneNetMqttConnected() || (now - g_lastReportMs) < PROPERTY_REPORT_INTERVAL_MS) // 未连接或未到周期
    {                                                                                   // 分支开始
        return;                                                                         // 直接返回
    } // 分支结束

    g_lastReportMs = now; // 记录本次上报时间

    const int32_t nearFullThreshold = g_fullWeightG * 9 / 10;                                               // 接近满载阈值
    const BoxBinData bin1 = {                                                                               // 1号箱数据
                             g_currentWeight1,                                                              // 当前重量
                             clampPercent(g_currentWeight1),                                                // 当前百分比
                             (g_currentWeight1 >= nearFullThreshold) && (g_currentWeight1 < g_fullWeightG), // 是否接近满载
                             g_currentWeight1 >= g_fullWeightG};                                            // 是否满载
    const BoxBinData bin2 = {                                                                               // 2号箱数据
                             g_currentWeight2,                                                              // 当前重量
                             clampPercent(g_currentWeight2),                                                // 当前百分比
                             (g_currentWeight2 >= nearFullThreshold) && (g_currentWeight2 < g_fullWeightG), // 是否接近满载
                             g_currentWeight2 >= g_fullWeightG};                                            // 是否满载
    const BoxBinData bin3 = {                                                                               // 3号箱数据
                             g_currentWeight3,                                                              // 当前重量
                             clampPercent(g_currentWeight3),                                                // 当前百分比
                             (g_currentWeight3 >= nearFullThreshold) && (g_currentWeight3 < g_fullWeightG), // 是否接近满载
                             g_currentWeight3 >= g_fullWeightG};                                            // 是否满载

    oneNetMqttUploadProperties(bin1, bin2, bin3, g_fullWeightG, g_aiConfThreshold); // 上报属性
} // 函数结束

void reportPropertiesNow()      // 立即上报属性
{                               // 函数开始
    if (!oneNetMqttConnected()) // 未连接就退出
    {                           // 分支开始
        return;                 // 直接返回
    } // 分支结束

    const int32_t nearFullThreshold = g_fullWeightG * 9 / 10;                                               // 接近满载阈值
    const BoxBinData bin1 = {                                                                               // 1号箱数据
                             g_currentWeight1,                                                              // 当前重量
                             clampPercent(g_currentWeight1),                                                // 当前百分比
                             (g_currentWeight1 >= nearFullThreshold) && (g_currentWeight1 < g_fullWeightG), // 是否接近满载
                             g_currentWeight1 >= g_fullWeightG};                                            // 是否满载
    const BoxBinData bin2 = {                                                                               // 2号箱数据
                             g_currentWeight2,                                                              // 当前重量
                             clampPercent(g_currentWeight2),                                                // 当前百分比
                             (g_currentWeight2 >= nearFullThreshold) && (g_currentWeight2 < g_fullWeightG), // 是否接近满载
                             g_currentWeight2 >= g_fullWeightG};                                            // 是否满载
    const BoxBinData bin3 = {                                                                               // 3号箱数据
                             g_currentWeight3,                                                              // 当前重量
                             clampPercent(g_currentWeight3),                                                // 当前百分比
                             (g_currentWeight3 >= nearFullThreshold) && (g_currentWeight3 < g_fullWeightG), // 是否接近满载
                             g_currentWeight3 >= g_fullWeightG};                                            // 是否满载

    oneNetMqttUploadProperties(bin1, bin2, bin3, g_fullWeightG, g_aiConfThreshold); // 立即上报
} // 函数结束
