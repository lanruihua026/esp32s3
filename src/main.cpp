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
    if (!g_prefsOk)
    {
        Serial.println("[NVS] Preferences begin failed; using defaults, persistence disabled");
    }
    if (g_prefsOk)
    {
        g_fullWeightG = gPrefs.getInt("full_w", 1000);
        g_aiConfThreshold = gPrefs.getFloat("ai_conf", 0.0f);
    }
    // 运行时保护：NVS 历史值若超出合法范围 [100, 5000]，重置为默认值，避免异常配置干扰告警逻辑
    if (g_fullWeightG < 100 || g_fullWeightG > 5000)
    {
        Serial.printf("[CONFIG] NVS full_w=%d out of range [100,5000], reset to 1000 g\n", g_fullWeightG);
        g_fullWeightG = 1000;
    }
    if (!(g_aiConfThreshold >= 0.0f && g_aiConfThreshold <= 1.0f))
    {
        Serial.printf("[CONFIG] NVS ai_conf=%.4f out of range [0,1], reset to 0.0000\n", g_aiConfThreshold);
        g_aiConfThreshold = 0.0f;
        if (g_prefsOk)
        {
            gPrefs.putFloat("ai_conf", g_aiConfThreshold);
        }
    }
    Serial.printf("[CONFIG] overflow_threshold_g=%d g, ai_conf_threshold=%.4f (%s)\n",
                  g_fullWeightG, g_aiConfThreshold, g_prefsOk ? "from NVS" : "defaults");

    initBoardIndicators();
    initCameraUart();
    initOledModule();
    startWiFiConnect();
    initHx711Modules();
    initWiFiWithTimeout();
    initServoModule();

    setInitModuleStatus(INIT_MODULE_BUTTON, INIT_RUNNING, "IRQ");
    showBootProgress(84, "Buttons");
    initButtonsForOledNavigation();
    setButton3Callback(silenceOverflowAlarmBuzzer);
    setInitModuleStatus(INIT_MODULE_BUTTON, INIT_OK, "Ready");

    initMqttModule();

    showBootProgress(100, "Starting");
    delay(200);

    initTimers();
    syncRuntimeHealth();
}

void loop()
{
    const uint32_t now = millis();

    pollHx711SerialCommands(gPrefs, g_prefsOk, g_currentWeight1, g_currentWeight2, g_currentWeight3);
    pollCameraUart();
    expireAiStateIfStale(now);
    updateServoByAiResult();
    pollButtons();

    tryReconnectWiFi(now);
    oneNetMqttLoop();
    processDeferredPropertyReport();
    syncRuntimeHealth();

    if ((now - g_lastOledUpdateMs) >= OLED_MIN_UPDATE_INTERVAL_MS)
    {
        updateOLEDDisplay();
        g_lastOledUpdateMs = now;
    }

    if ((now - g_lastSampleMs) >= WEIGHT_SAMPLE_INTERVAL_MS)
    {
        g_lastSampleMs = now;
        updateWeightsAndAlarm();
    }

    uploadPropertiesIfNeeded(now);
}
