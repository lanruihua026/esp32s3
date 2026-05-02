#include <Arduino.h>
#include "appState.h"
#include "binControl.h"
#include "buttonControl.h"
#include "cameraAi.h"
#include "connectToWiFi.h"
#include "hx711.h"
#include "oledInit.h"
#include "onenetMqtt.h"
#include "systemInit.h"

void setup()
{
    Serial.begin(115200);
    g_prefsOk = gPrefs.begin("sysconf", false);

    // 云端下发的阈值先从 NVS 恢复；越界值会回退到默认安全值
    if (g_prefsOk)
    {
        g_fullWeightG = gPrefs.getInt("full_w", 1000);
        g_aiConfThreshold = gPrefs.getFloat("ai_conf", 0.0f);
    }

    // 满溢阈值必须处于传感器量程范围内，异常值不参与后续告警判断。
    if (g_fullWeightG < 100 || g_fullWeightG > 5000)
    {
        g_fullWeightG = 1000;
    }
    // AI 置信度阈值来自云端下发，限定在 0~1，避免错误配置让舵机永久不动作。
    if (!(g_aiConfThreshold >= 0.0f && g_aiConfThreshold <= 1.0f))
    {
        g_aiConfThreshold = 0.0f;
        if (g_prefsOk)
        {
            gPrefs.putFloat("ai_conf", g_aiConfThreshold);
        }
    }

    initBoardIndicators();
    initCameraUart();
    initOledModule();
    // WiFi 连接先发起但不立即阻塞，给 HX711 预热和 OLED 初始化留出并行时间。
    startWiFiConnect();
    initHx711Modules();
    // 等待 WiFi 的总时间受系统配置约束，超时后仍允许设备进入本地运行态。
    initWiFiWithTimeout();
    initServoModule();

    // 按键依赖中断，单独在启动进度中标识，便于确认 OLED 翻页与消警按键可用。
    setInitModuleStatus(INIT_MODULE_BUTTON, INIT_RUNNING, "IRQ");
    showBootProgress(84, "Buttons");
    initButtonsForOledNavigation();
    setButton3Callback(silenceOverflowAlarmBuzzer);
    setInitModuleStatus(INIT_MODULE_BUTTON, INIT_OK, "Ready");

    // MQTT 在 WiFi 与基础外设之后初始化，避免网络回调过早访问未就绪的运行状态。
    initMqttModule();
    showBootProgress(100, "Starting");
    delay(200);

    initTimers();        // 初始化定时器，启动周期性任务调度
    syncRuntimeHealth(); // 同步系统当前运行状态到MQTT和OLED
}

void loop()
{
    const uint32_t now = millis();

    // 高实时性的本地链路先处理，随后处理网络与云端上报。
    pollHx711SerialCommands(gPrefs, g_prefsOk, g_currentWeight1, g_currentWeight2, g_currentWeight3);
    pollCameraUart();
    // 相机或 AI 服务超时后主动收敛状态，防止 OLED 和前端长期显示旧识别结果。
    expireAiStateIfStale(now);
    // 舵机控制放在串口解析之后，保证本轮新识别结果能尽快作用到执行机构。
    updateServoByAiResult();
    pollButtons();

    // 网络相关逻辑放在本地控制之后，避免 MQTT 阻塞影响称重、按键和舵机响应。
    tryReconnectWiFi(now);
    oneNetMqttLoop();
    processDeferredPropertyReport();
    syncRuntimeHealth();

    // OLED 刷新较慢，按最小间隔节流，避免清屏闪烁和 I2C 过度占用。
    if ((now - g_lastOledUpdateMs) >= OLED_MIN_UPDATE_INTERVAL_MS)
    {
        updateOLEDDisplay();
        g_lastOledUpdateMs = now;
    }

    // 称重采样与声光告警绑定，按固定周期执行，确保三路重量判断节奏一致。
    if ((now - g_lastSampleMs) >= WEIGHT_SAMPLE_INTERVAL_MS)
    {
        g_lastSampleMs = now;
        updateWeightsAndAlarm();
    }
    // 属性上报内部会检查 MQTT 状态和上报间隔，主循环只负责持续调度。
    uploadPropertiesIfNeeded(now);
}
