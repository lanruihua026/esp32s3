#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <cstdlib>
#include <cstring>

#include "buttonControl.h"
#include "buzzer.h"
#include "connectToWiFi.h"
#include "hx711.h"
#include "hx711_2.h"
#include "hx711_3.h"
#include "oledInit.h"
#include "onenetMqtt.h"
#include "servoControl.h"

namespace
{
    // ===== 硬件资源定义 =====
    // 这里集中放引脚、时间参数、业务阈值，便于后续调试和统一修改。
    constexpr uint8_t WARNING_LIGHT_PIN = 6;
    constexpr uint8_t RGB_LED_PIN = 38;
    constexpr uint8_t RGB_LED_COUNT = 1;

    constexpr uint8_t CAM_UART_RX_PIN = 18;
    constexpr uint8_t CAM_UART_TX_PIN = 17;
    constexpr uint32_t CAM_UART_BAUD = 115200;

    constexpr int32_t FULL_WEIGHT_G = 1000;
    constexpr int32_t WARNING_RELEASE_HYSTERESIS_G = 100;
    constexpr uint32_t WEIGHT_SAMPLE_INTERVAL_MS = 500;
    constexpr uint32_t PROPERTY_REPORT_INTERVAL_MS = 10000;
    constexpr uint32_t WIFI_RETRY_INTERVAL_MS = 15000;
    constexpr uint32_t WIFI_BOOT_WAIT_MS = 10000;

    // Recalibrated with a 216 g reference weight:
    // B1: 229 g -> 216 g (fine tune), B2: 480 g -> 216 g, B3: 483 g -> 216 g.
    constexpr float HX711_CAL_FACTOR_1 = 452.3f;
    constexpr float HX711_CAL_FACTOR_2 = 419.8f;
    constexpr float HX711_CAL_FACTOR_3 = 455.5f;

    constexpr const char *ONENET_PRODUCT_ID = "f45hkc7xC7";
    constexpr const char *ONENET_DEVICE_NAME = "Box1";
    constexpr const char *ONENET_BASE64_KEY = "T0R5ejYyM1JrT2VuczBkZllINmZuazRicEMxc29xcnk=";

    // 板载 RGB 仅用于上电后快速关闭，避免默认乱闪。
    Adafruit_NeoPixel boardRgb(RGB_LED_COUNT, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);
    // UART1 用来接收 ESP32-CAM 识别结果。
    HardwareSerial CameraUart(1);

    // 摄像头串口是按“逐字节接收，按行解析”的方式处理的，
    // 这里是接收缓存区。
    char g_camLineBuf[128] = {0};
    size_t g_camLinePos = 0;

    // 最近一次 AI 识别结果缓存：
    // 供舵机分拣、OLED 显示使用。
    bool g_lastAiDetected = false;
    char g_lastAiLabel[32] = "none";
    float g_lastAiConf = 0.0f;
    uint32_t g_lastAiUpdateMs = 0;

    // 三个仓位的最新重量缓存：
    // 供告警、OLED 页面、OneNET 上报复用。
    int32_t g_currentWeight1 = 0;
    int32_t g_currentWeight2 = 0;
    int32_t g_currentWeight3 = 0;

    // 周期任务时间戳：
    // 分别用于属性上报、重量采样、WiFi 断线重连。
    uint32_t g_lastReportMs = 0;
    uint32_t g_lastSampleMs = 0;
    uint32_t g_lastWiFiRetryMs = 0;

    // 启动阶段和运行阶段的状态缓存。
    bool g_hx711_1InitOk = false;
    bool g_hx711_2InitOk = false;
    bool g_hx711_3InitOk = false;
    bool g_wifiInitTimeout = false;
    bool g_warningActive = false;

    // 将重量转换为百分比，范围限制在 0~100%，
    // 用于云平台物模型上报。
    float clampPercent(int32_t weight)
    {
        float percent = (weight * 100.0f) / static_cast<float>(FULL_WEIGHT_G);
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

    bool allHx711Ready()
    {
        return g_hx711_1InitOk && g_hx711_2InitOk && g_hx711_3InitOk;
    }

    // 将当前运行状态同步给 OLED 状态页。
    // WiFi 一旦后续恢复成功，会自动清除“启动超时”标记。
    void syncRuntimeHealth()
    {
        const bool wifiConnected = (WiFi.status() == WL_CONNECTED);
        if (wifiConnected)
        {
            g_wifiInitTimeout = false;
        }

        setRuntimeHealth(wifiConnected, g_wifiInitTimeout, allHx711Ready(), oneNetMqttConnected());
    }

    void initBoardIndicators()
    {
        // 先关掉上电后可能乱亮的灯和蜂鸣器，避免误判为故障。
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
        // 初始化与 ESP32-CAM 相连的串口，用来接收识别结果。
        CameraUart.begin(CAM_UART_BAUD, SERIAL_8N1, CAM_UART_RX_PIN, CAM_UART_TX_PIN);
    }

    void initOled()
    {
        // OLED 放在前面初始化，后续模块启动进度都显示在这里。
        resetInitModuleStatus();
        setInitModuleStatus(INIT_MODULE_OLED, INIT_RUNNING, "Init");
        setupOLED();
        setInitModuleStatus(INIT_MODULE_OLED, isOLEDReady() ? INIT_OK : INIT_ERROR, isOLEDReady() ? "Ready" : "Fail");
        setFullWeight(FULL_WEIGHT_G);
    }

    void initHx711Modules()
    {
        // 依次初始化三路重量传感器，并写入各自校准系数。
        // 三路全部可用才认为称重模块整体正常。
        setInitModuleStatus(INIT_MODULE_HX711, INIT_RUNNING, "Probe");

        char dbgBuf[32];
        float probeRaw;
        int probeValid;

        // --- HX711_1 ---
        showBootProgress(20, "HX711_1 probing...");
        g_hx711_1InitOk = setupHX711();
        hx711GetProbeResult(&probeRaw, &probeValid);
        snprintf(dbgBuf, sizeof(dbgBuf), "HX711_1:%s v=%d/5 r=%.0f",
                 g_hx711_1InitOk ? "OK" : "FAIL", probeValid, probeRaw);
        showBootProgress(24, dbgBuf);
        Serial.println(dbgBuf);
        delay(2000);

        setCalibrationFactor(HX711_CAL_FACTOR_1);

        // --- HX711_2 ---
        showBootProgress(30, "HX711_2 probing...");
        g_hx711_2InitOk = setupHX711_2();
        hx711GetProbeResult2(&probeRaw, &probeValid);
        snprintf(dbgBuf, sizeof(dbgBuf), "HX711_2:%s v=%d/5 r=%.0f",
                 g_hx711_2InitOk ? "OK" : "FAIL", probeValid, probeRaw);
        showBootProgress(34, dbgBuf);
        Serial.println(dbgBuf);
        delay(2000);

        setCalibrationFactor2(HX711_CAL_FACTOR_2);

        // --- HX711_3 ---
        showBootProgress(38, "HX711_3 probing...");
        g_hx711_3InitOk = setupHX711_3();
        hx711GetProbeResult3(&probeRaw, &probeValid);
        snprintf(dbgBuf, sizeof(dbgBuf), "HX711_3:%s v=%d/5 r=%.0f",
                 g_hx711_3InitOk ? "OK" : "FAIL", probeValid, probeRaw);
        showBootProgress(42, dbgBuf);
        Serial.println(dbgBuf);
        delay(2000);

        setCalibrationFactor3(HX711_CAL_FACTOR_3);

        setInitModuleStatus(INIT_MODULE_HX711, allHx711Ready() ? INIT_OK : INIT_ERROR, allHx711Ready() ? "Ready" : "Check");
    }

    void initWiFiWithTimeout()
    {
        // WiFi 允许在启动阶段等待一段时间，
        // 超时也继续进入系统，避免整机卡死在联网阶段。
        setInitModuleStatus(INIT_MODULE_WIFI, INIT_RUNNING, "Connecting");
        showBootProgress(45, "WiFi");
        setupWiFi();

        const uint32_t startMs = millis();
        while (WiFi.status() != WL_CONNECTED && (millis() - startMs) < WIFI_BOOT_WAIT_MS)
        {
            delay(200);
            const uint32_t elapsed = millis() - startMs;
            const uint8_t progress = 45 + static_cast<uint8_t>((elapsed * 20UL) / WIFI_BOOT_WAIT_MS);
            showBootProgress(progress, "WiFi");
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
            setInitModuleStatus(INIT_MODULE_WIFI, INIT_TIMEOUT, "10s timeout");
            showBootProgress(65, "WiFi Timeout");
        }
    }

    void initServoModule()
    {
        // 舵机属于执行器，放在网络之后初始化，
        // 这样供电更稳定，且自检动作更容易观察。
        setInitModuleStatus(INIT_MODULE_SERVO, INIT_RUNNING, "SelfTest");
        showBootProgress(72, "Servo");
        initServo();
        runServoSelfTest();
        setInitModuleStatus(INIT_MODULE_SERVO, INIT_OK, "Ready");
    }

    void initButtons()
    {
        // 按键只负责切换 OLED 页面，不参与核心控制逻辑。
        setInitModuleStatus(INIT_MODULE_BUTTON, INIT_RUNNING, "IRQ");
        showBootProgress(84, "Buttons");
        setupButtons();
        setButton1Callback([]()
                           { prevOledPage(); });
        setButton2Callback([]()
                           { toggleOledPage(); });
        setInitModuleStatus(INIT_MODULE_BUTTON, INIT_OK, "Ready");
    }

    void initMqtt()
    {
        // MQTT 这里只做参数配置，真正连接在 loop() 中由 oneNetMqttLoop() 维护。
        setInitModuleStatus(INIT_MODULE_MQTT, INIT_RUNNING, "Config");
        showBootProgress(92, "OneNET");

        const OneNetMqttConfig cfg = {
            "mqtts.heclouds.com",
            1883,
            ONENET_PRODUCT_ID,
            ONENET_DEVICE_NAME,
            ONENET_BASE64_KEY,
            1893456000,
            OneNetSignMethod::SHA256};

        oneNetMqttBegin(cfg);
        setInitModuleStatus(INIT_MODULE_MQTT, INIT_OK, "Ready");
    }

    void initTimers()
    {
        // 所有周期任务都从系统启动完成时开始计时。
        const uint32_t now = millis();
        g_lastReportMs = now;
        g_lastSampleMs = now;
        g_lastWiFiRetryMs = now;
    }

    void setAiState(bool detected, const char *label, float conf)
    {
        // 统一更新 AI 识别缓存，避免串口解析逻辑和显示逻辑分散。
        g_lastAiDetected = detected;
        g_lastAiConf = conf;
        g_lastAiUpdateMs = millis();

        strncpy(g_lastAiLabel, label, sizeof(g_lastAiLabel) - 1);
        g_lastAiLabel[sizeof(g_lastAiLabel) - 1] = '\0';

        setAiResult(detected, g_lastAiLabel, g_lastAiConf, g_lastAiUpdateMs);
    }

    void parseCameraLine(const char *line)
    {
        // ESP32-CAM 当前串口协议：
        // 1. DET,<label>,<conf> 识别到目标
        // 2. NONE               当前帧没有目标
        // 3. ERR,<msg>          摄像头侧或网络侧异常
        if (line == nullptr || line[0] == '\0')
        {
            return;
        }

        if (strncmp(line, "DET,", 4) == 0)
        {
            const char *p1 = strchr(line, ',');
            const char *p2 = p1 ? strchr(p1 + 1, ',') : nullptr;
            if (!p1 || !p2)
            {
                return;
            }

            char label[sizeof(g_lastAiLabel)] = {0};
            size_t labelLen = static_cast<size_t>(p2 - (p1 + 1));
            if (labelLen >= sizeof(label))
            {
                labelLen = sizeof(label) - 1;
            }
            memcpy(label, p1 + 1, labelLen);

            setAiState(true, label, static_cast<float>(atof(p2 + 1)));
            return;
        }

        if (strcmp(line, "NONE") == 0)
        {
            setAiState(false, "none", 0.0f);
            return;
        }

        if (strncmp(line, "ERR,", 4) == 0)
        {
            setAiState(false, "ERR", 0.0f);
            Serial.printf("[CAM] %s\n", line);
        }
    }

    void pollCameraUart()
    {
        // 持续读取摄像头串口。
        // 读到换行就认为收到一条完整消息，交给 parseCameraLine() 解析。
        while (CameraUart.available())
        {
            const char c = static_cast<char>(CameraUart.read());
            if (c == '\r')
            {
                continue;
            }

            if (c == '\n')
            {
                g_camLineBuf[g_camLinePos] = '\0';
                parseCameraLine(g_camLineBuf);
                g_camLinePos = 0;
                continue;
            }

            if (g_camLinePos < sizeof(g_camLineBuf) - 1)
            {
                g_camLineBuf[g_camLinePos++] = c;
            }
            else
            {
                g_camLinePos = 0;
            }
        }
    }

    int servoAngleForLabel(const char *label)
    {
        // 将识别类别映射到舵机角度，本质上就是“不同垃圾 -> 不同仓位”。
        if (strcmp(label, "MobilePhone") == 0)
        {
            return 0;
        }
        if (strcmp(label, "Charger") == 0)
        {
            return 45;
        }
        if (strcmp(label, "Battery") == 0)
        {
            return 90;
        }
        if (strcmp(label, "Earphone") == 0)
        {
            return 180;
        }
        return 0;
    }

    void updateServoByAiResult()
    {
        // 识别到目标时按类别转到对应仓位；
        // 没识别到时让舵机回到默认位置。
        setServoAngle(g_lastAiDetected ? servoAngleForLabel(g_lastAiLabel) : 0);
    }

    void updateWeightsAndAlarm()
    {
        // 周期读取三路称重值，并同步到 OLED。
        // 目前告警只针对 1 号仓位：超阈值亮灯并鸣叫，低于回差后解除。
        g_currentWeight1 = static_cast<int32_t>(getWeight());
        g_currentWeight2 = static_cast<int32_t>(getWeight2());
        g_currentWeight3 = static_cast<int32_t>(getWeight3());

        setCurrentWeights(g_currentWeight1, g_currentWeight2, g_currentWeight3);

        if (!g_warningActive && g_currentWeight1 >= FULL_WEIGHT_G)
        {
            g_warningActive = true;
            digitalWrite(WARNING_LIGHT_PIN, HIGH);
            buzzerOn();
        }
        else if (g_warningActive && g_currentWeight1 < (FULL_WEIGHT_G - WARNING_RELEASE_HYSTERESIS_G))
        {
            g_warningActive = false;
            digitalWrite(WARNING_LIGHT_PIN, LOW);
            buzzerOff();
        }
    }

    void tryReconnectWiFi(uint32_t now)
    {
        // 启动后如果 WiFi 断开，按固定周期重试，避免频繁重连。
        if (WiFi.status() == WL_CONNECTED || (now - g_lastWiFiRetryMs) < WIFI_RETRY_INTERVAL_MS)
        {
            return;
        }

        g_lastWiFiRetryMs = now;
        WiFi.disconnect(false, false);
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }

    void uploadPropertiesIfNeeded(uint32_t now)
    {
        // 周期把三路重量、百分比、是否满载上传到 OneNET。
        // 只有 MQTT 已连接时才会执行上报。
        if (!oneNetMqttConnected() || (now - g_lastReportMs) < PROPERTY_REPORT_INTERVAL_MS)
        {
            return;
        }

        g_lastReportMs = now;

        const BoxBinData bin1 = {g_currentWeight1, clampPercent(g_currentWeight1), g_currentWeight1 >= FULL_WEIGHT_G};
        const BoxBinData bin2 = {g_currentWeight2, clampPercent(g_currentWeight2), g_currentWeight2 >= FULL_WEIGHT_G};
        const BoxBinData bin3 = {g_currentWeight3, clampPercent(g_currentWeight3), g_currentWeight3 >= FULL_WEIGHT_G};

        oneNetMqttUploadProperties(bin1, bin2, bin3);
    }
} // namespace

void setup()
{
    Serial.begin(115200);

    // 初始化顺序遵循嵌入式常见思路：
    // 先基础外设，再显示，再传感器，再网络，再执行器，最后云端和交互。
    initBoardIndicators();
    initCameraUart();
    initOled();
    initHx711Modules();
    initWiFiWithTimeout();
    initServoModule();
    initButtons();
    initMqtt();

    showBootProgress(100, "Starting");
    delay(200);

    initTimers();
    syncRuntimeHealth();
    Serial.println("System ready");
}

void loop()
{
    const uint32_t now = millis();

    // 主循环按“先采集输入，再更新执行器，最后处理周期任务”组织。
    pollCameraUart();
    updateServoByAiResult();
    pollButtons();

    tryReconnectWiFi(now);
    oneNetMqttLoop();
    syncRuntimeHealth();
    updateOLEDDisplay();

    if ((now - g_lastSampleMs) >= WEIGHT_SAMPLE_INTERVAL_MS)
    {
        g_lastSampleMs = now;
        updateWeightsAndAlarm();
    }

    uploadPropertiesIfNeeded(now);
}
