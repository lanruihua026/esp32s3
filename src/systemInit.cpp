#include "systemInit.h"

#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>
#include <cstdio>
#include <cstring>

#include "appState.h"
#include "binControl.h"
#include "buttonControl.h"
#include "buzzer.h"
#include "connectToWiFi.h"
#include "hx711.h"
#include "hx711_2.h"
#include "hx711_3.h"
#include "hx711BootConfig.h"
#include "oledInit.h"
#include "onenetMqtt.h"
#include "secrets.h"
#include "servoControl.h"

namespace
{
    Adafruit_NeoPixel boardRgb(RGB_LED_COUNT, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);
    bool g_propertyReportPending = false;

    bool allHx711Ready()
    {
        return g_hx711_1InitOk && g_hx711_2InitOk && g_hx711_3InitOk;
    }

    void onPropertySet(const char *payload, unsigned int len)
    {
        char buf[512] = {0};
        unsigned int copyLen = (len < sizeof(buf) - 1) ? len : sizeof(buf) - 1;
        memcpy(buf, payload, copyLen);

        JsonDocument doc;
        if (deserializeJson(doc, buf) != DeserializationError::Ok)
        {
            return;
        }

        bool changed = false;

        JsonVariant pOver = doc["params"]["overflow_threshold_g"];
        if (!pOver.isNull())
        {
            JsonVariant vOver = pOver["value"];
            if (vOver.isNull())
            {
                vOver = pOver;
            }
            int32_t newThreshold = vOver.as<int32_t>();
            if (newThreshold >= 100 && newThreshold <= 5000)
            {
                g_fullWeightG = newThreshold;
                if (g_prefsOk)
                {
                    gPrefs.putInt("full_w", newThreshold);
                }
                setFullWeight(g_fullWeightG);
                changed = true;
            }
        }

        JsonVariant pAi = doc["params"]["ai_conf_threshold"];
        if (!pAi.isNull())
        {
            JsonVariant vAi = pAi["value"];
            if (vAi.isNull())
            {
                vAi = pAi;
            }
            float newAi = vAi.as<float>();
            if (newAi >= 0.0f && newAi <= 1.0f)
            {
                g_aiConfThreshold = newAi;
                if (g_prefsOk)
                {
                    gPrefs.putFloat("ai_conf", newAi);
                }
                changed = true;
            }
        }

        if (changed)
        {
            // MQTT 回调内只标记“需要补上报”，由主循环异步发送，避免在回调栈内嵌套 publish。
            g_propertyReportPending = true;
        }
    }
}

void syncRuntimeHealth()
{
    static bool s_prevWifiConnected = false;

    const bool wifiConnected = (WiFi.status() == WL_CONNECTED);

    if (s_prevWifiConnected && !wifiConnected)
    {
        oneNetMqttDisconnect();
    }
    s_prevWifiConnected = wifiConnected;

    if (wifiConnected)
    {
        g_wifiInitTimeout = false;
    }

    setRuntimeHealth(wifiConnected, g_wifiInitTimeout, allHx711Ready(), oneNetMqttConnected());
    setHx711ChannelReady(g_hx711_1InitOk, g_hx711_2InitOk, g_hx711_3InitOk);
    setAiConfThreshold(g_aiConfThreshold);
}

void processDeferredPropertyReport()
{
    if (!g_propertyReportPending || !oneNetMqttConnected())
    {
        return;
    }

    g_propertyReportPending = false;
    reportPropertiesNow();
}

void initBoardIndicators()
{
    boardRgb.begin();
    boardRgb.setPixelColor(0, 0);
    boardRgb.show();

    pinMode(WARNING_LIGHT_PIN, OUTPUT);
    digitalWrite(WARNING_LIGHT_PIN, LOW);

    buzzerInit();
    buzzerOff();
}

void initCameraUart()
{
    CameraUart.begin(CAM_UART_BAUD, SERIAL_8N1, CAM_UART_RX_PIN, CAM_UART_TX_PIN);
}

void initOledModule()
{
    resetInitModuleStatus();
    setInitModuleStatus(INIT_MODULE_OLED, INIT_RUNNING, "Init");
    setupOLED();
    setInitModuleStatus(INIT_MODULE_OLED, isOLEDReady() ? INIT_OK : INIT_ERROR, isOLEDReady() ? "Ready" : "Fail");
    setFullWeight(g_fullWeightG);
}

void initHx711Modules()
{
    setInitModuleStatus(INIT_MODULE_HX711, INIT_RUNNING, "Probe");

    if (HX711_BOOT_STRATEGY == 1)
    {
        showBootProgress(18, "HX711 warm");
        delay(HX711_GLOBAL_WARMUP_MS);
    }

    showBootProgress(20, "HX711_1");
    g_hx711_1InitOk = initHx711Channel1(gPrefs, g_prefsOk, HX711_CAL_FACTOR_1, HX711_BOOT_EMPTY_THRESHOLD_G);
    showBootProgress(28, "HX711_1 Cal");

    showBootProgress(32, "HX711_2");
    g_hx711_2InitOk = initHx711Channel2(gPrefs, g_prefsOk, HX711_CAL_FACTOR_2, HX711_BOOT_EMPTY_THRESHOLD_G);
    showBootProgress(36, "HX711_2 Cal");

    showBootProgress(38, "HX711_3");
    g_hx711_3InitOk = initHx711Channel3(gPrefs, g_prefsOk, HX711_CAL_FACTOR_3, HX711_BOOT_EMPTY_THRESHOLD_G);
    showBootProgress(40, "HX711_3 Cal");

    setInitModuleStatus(INIT_MODULE_HX711, allHx711Ready() ? INIT_OK : INIT_ERROR, allHx711Ready() ? "Ready" : "Check");
}

void initWiFiWithTimeout()
{
    setInitModuleStatus(INIT_MODULE_WIFI, INIT_RUNNING, "Connecting");
    showBootProgress(45, "WiFi");

    if (!wifiBootConnectStarted())
    {
        startWiFiConnect();
    }

    const uint32_t startMs = wifiBootConnectStartMillis();
    while (WiFi.status() != WL_CONNECTED)
    {
        const uint32_t elapsed = millis() - startMs;
        if (elapsed >= WIFI_BOOT_WAIT_MS)
        {
            break;
        }
        delay(200);
        const uint8_t progress = 45 + static_cast<uint8_t>((elapsed * 20UL) / WIFI_BOOT_WAIT_MS);
        char wifiLine[17];
        snprintf(wifiLine, sizeof(wifiLine), "WiFi %lus", static_cast<unsigned long>(elapsed / 1000UL));
        showBootProgress(progress, wifiLine);
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        g_wifiInitTimeout = false;
        setInitModuleStatus(INIT_MODULE_WIFI, INIT_OK, "Connected");
        showBootProgress(65, "WiFi Ready");
    }
    else
    {
        g_wifiInitTimeout = true;
        char detail[17];
        const unsigned long sec = WIFI_BOOT_WAIT_MS / 1000UL;
        snprintf(detail, sizeof(detail), "%lus timeout", sec);
        setInitModuleStatus(INIT_MODULE_WIFI, INIT_TIMEOUT, detail);
        showBootProgress(65, "WiFi Timeout");
    }
}

void initServoModule()
{
    setInitModuleStatus(INIT_MODULE_SERVO, INIT_RUNNING, "SelfTest");
    showBootProgress(72, "Srv Init");
    initServo();
    runServoSelfTest([](uint8_t channel)
                     {
                         if (!isOLEDReady())
                         {
                             return;
                         }
                         if (channel == 0)
                         {
                             showBootProgress(72, "Srv Home");
                             return;
                         }
                         char line[17];
                         snprintf(line, sizeof(line), "Servo %u", static_cast<unsigned>(channel));
                         showBootProgress(72, line);
                     });
    setInitModuleStatus(INIT_MODULE_SERVO, INIT_OK, "Ready");
}

void initMqttModule()
{
    setInitModuleStatus(INIT_MODULE_MQTT, INIT_RUNNING, "Config");
    showBootProgress(92, "OneNET");

    const OneNetMqttConfig cfg = {
        "mqtts.heclouds.com",
        1883,
        ONENET_PRODUCT_ID,
        ONENET_DEVICE_NAME,
        ONENET_BASE64_KEY,
        ONENET_TOKEN_EXPIRE_AT,
        OneNetSignMethod::SHA256};

    oneNetMqttBegin(cfg);
    oneNetSetPropertySetCallback(onPropertySet);
    setInitModuleStatus(INIT_MODULE_MQTT, INIT_OK, "Ready");
}

void initTimers()
{
    const uint32_t now = millis();
    g_lastReportMs = now;
    g_lastSampleMs = now;
    g_lastWiFiRetryMs = now;
}

void tryReconnectWiFi(uint32_t now)
{
    if (WiFi.status() == WL_CONNECTED || (now - g_lastWiFiRetryMs) < WIFI_RETRY_INTERVAL_MS)
    {
        return;
    }

    g_lastWiFiRetryMs = now;
    WiFi.disconnect(false, false);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}
