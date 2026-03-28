/**
 * @file main.cpp
 * @brief 智能垃圾分类箱主程序
 *
 * 系统功能概述：
 * 1. ESP32-CAM 通过串口将 AI 识别结果（垃圾类别+置信度）发送给本机。
 * 2. 本机根据识别类别驱动舵机将垃圾导入对应仓位。
 * 3. 三路 HX711 压力传感器分别监测三个仓位的重量，满载时触发声光告警。
 * 4. 重量数据通过 OneNET MQTT 定期上报至云平台，供远程监控使用。
 * 5. OLED 屏幕实时显示系统状态、识别结果和仓位重量，支持按键翻页。
 *
 * 初始化顺序：指示器 → 摄像头串口 → OLED → HX711 → WiFi → 舵机 → 按键 → MQTT
 */

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>
#include <Preferences.h>
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
    constexpr uint8_t WARNING_LIGHT_PIN = 6; // 满载告警灯 GPIO 引脚
    constexpr uint8_t RGB_LED_PIN = 38;      // 板载 WS2812 RGB LED 数据引脚
    constexpr uint8_t RGB_LED_COUNT = 1;     // 板载 RGB LED 数量（仅 1 颗）

    constexpr uint8_t CAM_UART_RX_PIN = 18;    // 接收 ESP32-CAM 数据的串口 RX 引脚
    constexpr uint8_t CAM_UART_TX_PIN = 17;    // 发送给 ESP32-CAM 的串口 TX 引脚（保留，当前未使用）
    constexpr uint32_t CAM_UART_BAUD = 115200; // 与 ESP32-CAM 通信的波特率

    // 满载阈值改为运行时变量，支持通过 OneNET MQTT property/set 动态修改并持久化到 NVS。
    // 初始值为默认值，setup() 中会从 NVS 加载用户上次保存的值。
    int32_t g_fullWeightG = 1000;
    // AI 置信度阈值（0~1）：高于此值才允许舵机分拣；0 表示不按置信度过滤（与升级前行为一致）。
    float g_aiConfThreshold = 0.0f;
    constexpr int32_t WARNING_RELEASE_HYSTERESIS_G = 100;  // 告警解除回差（克），低于 g_fullWeightG-100g 才解除，防止抖动反复触发
    constexpr uint32_t WEIGHT_SAMPLE_INTERVAL_MS = 500;    // 重量采样周期（毫秒）
    constexpr uint32_t PROPERTY_REPORT_INTERVAL_MS = 3000; // OneNET 属性上报周期（毫秒）：3s 上报，配合网页 2s 轮询显著提升数据时效性
    constexpr uint32_t WIFI_RETRY_INTERVAL_MS = 15000;     // WiFi 断线后重连尝试间隔（毫秒）
    constexpr uint32_t WIFI_BOOT_WAIT_MS = 5000;           // 启动阶段等待 WiFi 连接的最长时间（毫秒）：连上即退出，最长 5s

    // HX711 校准系数（raw ADC 差值 / 克），使用 216g 砝码标定。
    // 修改方法：new_factor = old_factor × (当前显示值 / 实际重量)
    constexpr float HX711_CAL_FACTOR_1 = 452.3f; // 1 号仓（GPIO1/2）
    constexpr float HX711_CAL_FACTOR_2 = 419.8f; // 2 号仓（GPIO11/12）
    constexpr float HX711_CAL_FACTOR_3 = 455.5f; // 3 号仓（GPIO13/14）

    // ===== OneNET 云平台配置 =====
    constexpr const char *ONENET_PRODUCT_ID = "f45hkc7xC7";                                   // 产品 ID
    constexpr const char *ONENET_DEVICE_NAME = "Box1";                                        // 设备名称
    constexpr const char *ONENET_BASE64_KEY = "T0R5ejYyM1JrT2VuczBkZllINmZuazRicEMxc29xcnk="; // Base64 编码的设备密钥

    // NVS 持久化存储，用于保存满载阈值与 AI 置信度阈值，跨重启保持用户设置。
    Preferences gPrefs;

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
    bool g_hx711_1InitOk = false;   // 1 号仓 HX711 是否初始化成功，业务含义是称重模块的 1 号通道是否正常。
    bool g_hx711_2InitOk = false;   // 2 号仓 HX711 是否初始化成功，业务含义是称重模块的 2 号通道是否正常。
    bool g_hx711_3InitOk = false;   // 3 号仓 HX711 是否初始化成功，业务含义是称重模块的 3 号通道是否正常。
    bool g_wifiInitTimeout = false; // 启动阶段 WiFi 是否连接超时，业务含义是系统是否处于“联网异常”状态，影响 OLED 显示和告警逻辑。
    bool g_warningActive = false;   // 告警状态，true 表示当前处于满载告警中，false 表示未告警或已解除告警。

    /**
     * @brief 将重量限制在 0~100% 的范围内
     * @param weight 重量（克）
     * @return 百分比
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

    /**
     * @brief 判断三路 HX711 是否全部初始化成功
     * @return true 三路均可用；false 至少一路失败
     * 业务含义：只有全部可用时，才认为称重模块整体正常，OLED 状态页才显示OK
     */
    bool allHx711Ready()
    {
        return g_hx711_1InitOk && g_hx711_2InitOk && g_hx711_3InitOk; // 只有三路都正常，才认为称重模块整体正常
    }

    /**
     * @brief 同步运行时系统状态
     */
    void syncRuntimeHealth()
    {
        const bool wifiConnected = (WiFi.status() == WL_CONNECTED); // 当前WiFi连接状态，用于检测联网是否异常
        if (wifiConnected)
        {
            g_wifiInitTimeout = false; // 只要当前 WiFi 连接正常，就认为没有联网异常，即使启动阶段曾经超时过也没关系了，OLED 显示和告警逻辑都以当前连接状态为准。
        }

        setRuntimeHealth(wifiConnected, g_wifiInitTimeout, allHx711Ready(), oneNetMqttConnected());
    }

    /**
     * @brief 上电初始化时默认关闭蜂鸣器，警示灯，板载RGB
     */
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

    /**
     * @brief 初始化摄像头串口，用于接收esp32cam回传的识别结果
     */
    void initCameraUart()
    {
        CameraUart.begin(CAM_UART_BAUD, SERIAL_8N1, CAM_UART_RX_PIN, CAM_UART_TX_PIN);
    }

    /**
     * @brief 初始化 OLED 显示屏
     */
    void initOled()
    {
        resetInitModuleStatus();
        setInitModuleStatus(INIT_MODULE_OLED, INIT_RUNNING, "Init");
        setupOLED();
        setInitModuleStatus(INIT_MODULE_OLED, isOLEDReady() ? INIT_OK : INIT_ERROR, isOLEDReady() ? "Ready" : "Fail");
        setFullWeight(g_fullWeightG);
    }

    /**
     * @brief 初始化 HX711 重量传感器模块
     */
    void initHx711Modules()
    {
        // 依次初始化三路重量传感器，并写入各自校准系数。
        // 三路全部可用才认为称重模块整体正常。
        setInitModuleStatus(INIT_MODULE_HX711, INIT_RUNNING, "Probe");

        showBootProgress(20, "HX711_1");
        g_hx711_1InitOk = setupHX711();
        showBootProgress(28, "HX711_1 Cal");
        setCalibrationFactor(HX711_CAL_FACTOR_1);

        showBootProgress(32, "HX711_2");
        g_hx711_2InitOk = setupHX711_2();
        showBootProgress(36, "HX711_2 Cal");
        setCalibrationFactor2(HX711_CAL_FACTOR_2);

        showBootProgress(38, "HX711_3");
        g_hx711_3InitOk = setupHX711_3();
        showBootProgress(40, "HX711_3 Cal");
        setCalibrationFactor3(HX711_CAL_FACTOR_3);

        setInitModuleStatus(INIT_MODULE_HX711, allHx711Ready() ? INIT_OK : INIT_ERROR, allHx711Ready() ? "Ready" : "Check");
    }

    /**
     * @brief 初始化 WiFi 模块，并在启动阶段等待连接（有超时机制）
     */
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
            // 将已等待时间线性映射到进度条 45%~65% 区间，让用户看到联网进度
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
    /**
     * @brief 初始化舵机模块，并执行自检
     */
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

    /**
     * @brief 初始化按键模块
     */
    void initButtons()
    {
        // 按键只负责切换 OLED 页面，不参与核心控制逻辑。
        setInitModuleStatus(INIT_MODULE_BUTTON, INIT_RUNNING, "IRQ");
        showBootProgress(84, "Buttons");
        setupButtons();
        setButton1Callback([]()
                           { prevOledPage(); }); // 按键 1：向左翻页（2→1→0→2）
        setButton2Callback([]()
                           { toggleOledPage(); }); // 按键 2：向右翻页（0→1→2→0）
        setInitModuleStatus(INIT_MODULE_BUTTON, INIT_OK, "Ready");
    }

    /**
     * @brief OneNET property/set 回调：解析平台下发的满载阈值、AI 置信度阈值并持久化
     *
     * OneJSON 示例：
     *   {"params":{"overflow_threshold_g":{"value":800},"ai_conf_threshold":{"value":0.65}}}
     * 两字段可单独或同时下发。
     */
    void onPropertySet(const char *payload, unsigned int len)
    {
        char buf[384] = {0};
        unsigned int copyLen = (len < sizeof(buf) - 1) ? len : sizeof(buf) - 1;
        memcpy(buf, payload, copyLen);

        JsonDocument doc;
        if (deserializeJson(doc, buf) != DeserializationError::Ok)
        {
            Serial.println("[CONFIG] property/set JSON parse error");
            return;
        }

        JsonVariant vOver = doc["params"]["overflow_threshold_g"]["value"];
        if (!vOver.isNull())
        {
            int32_t newThreshold = vOver.as<int32_t>();
            if (newThreshold < 100 || newThreshold > 5000)
            {
                Serial.printf("[CONFIG] overflow_threshold_g=%d out of range [100,5000], ignored\n", newThreshold);
            }
            else
            {
                g_fullWeightG = newThreshold;
                gPrefs.putInt("full_w", newThreshold);
                setFullWeight(g_fullWeightG);
                Serial.printf("[CONFIG] overflow_threshold_g updated to %d g\n", newThreshold);
            }
        }

        JsonVariant vAi = doc["params"]["ai_conf_threshold"]["value"];
        if (!vAi.isNull())
        {
            float newAi = vAi.as<float>();
            if (newAi < 0.0f || newAi > 1.0f)
            {
                Serial.printf("[CONFIG] ai_conf_threshold=%.4f out of range [0,1], ignored\n", newAi);
            }
            else
            {
                g_aiConfThreshold = newAi;
                gPrefs.putFloat("ai_conf", newAi);
                Serial.printf("[CONFIG] ai_conf_threshold updated to %.4f\n", newAi);
            }
        }
    }

    /**
     * @brief 初始化 MQTT 模块
     */
    void initMqtt()
    {
        // MQTT 这里只做参数配置，真正连接在 loop() 中由 oneNetMqttLoop() 维护。
        setInitModuleStatus(INIT_MODULE_MQTT, INIT_RUNNING, "Config");
        showBootProgress(92, "OneNET");

        const OneNetMqttConfig cfg = {
            "mqtts.heclouds.com",      // OneNET MQTT 服务器地址
            1883,                      // MQTT 端口（非 TLS）
            ONENET_PRODUCT_ID,         // 产品 ID
            ONENET_DEVICE_NAME,        // 设备名称
            ONENET_BASE64_KEY,         // Base64 编码的设备鉴权密钥
            1893456000,                // Token 过期时间戳（Unix 时间，需定期更新）
            OneNetSignMethod::SHA256}; // 签名算法：HMAC-SHA256

        oneNetMqttBegin(cfg);
        // 注册平台下发回调，使网页端满载阈值变更能同步到硬件
        oneNetSetPropertySetCallback(onPropertySet);
        setInitModuleStatus(INIT_MODULE_MQTT, INIT_OK, "Ready");
    }

    /**
     * @brief 初始化定时器
     */
    void initTimers()
    {
        // 所有周期任务都从系统启动完成时开始计时。
        const uint32_t now = millis();
        g_lastReportMs = now;
        g_lastSampleMs = now;
        g_lastWiFiRetryMs = now;
    }

    /**
     * @brief 更新模型识别结果，并同步更新 OLED 显示内容和 OneNET 上报缓存
     * @param detected 是否识别到目标
     * @param label 识别到的目标类别标签
     * @param conf 识别置信度（0~1）
     */
    void setAiState(bool detected, const char *label, float conf)
    {
        // 统一更新模型识别缓存，避免串口解析逻辑和显示逻辑分散。
        g_lastAiDetected = detected; // 是否识别到目标
        g_lastAiConf = conf;         // 识别置信度
        g_lastAiUpdateMs = millis(); // 识别结果更新时间戳，多少毫秒前被更新，用于 OLED 显示“刚识别”状态

        strncpy(g_lastAiLabel, label, sizeof(g_lastAiLabel) - 1); // 复制识别标签
        g_lastAiLabel[sizeof(g_lastAiLabel) - 1] = '\0';

        setAiResult(detected, g_lastAiLabel, g_lastAiConf, g_lastAiUpdateMs); // 同步更新 OLED 显示内容和 OneNET 上报缓存
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
            const char *p1 = strchr(line, ',');                  // 指向第一个逗号（标签起始前）
            const char *p2 = p1 ? strchr(p1 + 1, ',') : nullptr; // 指向第二个逗号（置信度起始前）
            if (!p1 || !p2)
            {
                return; // 报文格式不合法，丢弃
            }

            char label[sizeof(g_lastAiLabel)] = {0};
            size_t labelLen = static_cast<size_t>(p2 - (p1 + 1)); // 计算标签字符串长度
            if (labelLen >= sizeof(label))
            {
                labelLen = sizeof(label) - 1; // 防止标签过长溢出缓冲区
            }
            memcpy(label, p1 + 1, labelLen); // 从 p1+1 处拷贝 labelLen 个字节作为标签

            float conf = static_cast<float>(atof(p2 + 1));
            setAiState(true, label, conf);
            Serial.printf("[CAM] DET label=%s conf=%.1f%%\n", label, conf * 100.0f);
            return;
        }

        if (strcmp(line, "NONE") == 0)
        {
            setAiState(false, "none", 0.0f);
            Serial.println("[CAM] NONE");
            return;
        }

        if (strncmp(line, "ERR,", 4) == 0)
        {
            setAiState(false, "ERR", 0.0f);
            Serial.printf("[CAM] ERR %s\n", line + 4);
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
                // 行缓冲区溢出（报文异常过长），丢弃当前行，从头重新接收
                g_camLinePos = 0;
            }
        }
    }

    void updateServoByAiResult()
    {
        // 三路舵机各负责一个仓位，识别到对应类别时触发一次：转 180° 保持 2 秒后归位。
        // 同一时刻最多一个舵机处于工作状态，归位后须等检测结果清零才允许再次触发。
        // 仓位分配：
        //   Battery（电池）          → servo1 (GPIO7)  电池仓
        //   MobilePhone（手机）      → servo2 (GPIO8)  手机仓
        //   Charger / Earphone       → servo3 (GPIO16) 数码配件仓
        static const uint32_t SERVO_HOLD_MS = 2000; // 舵机保持 180° 的时长（毫秒）
        static int activeServo = 0;                 // 当前激活的舵机（0=无，1/2/3）
        static uint32_t triggerTimeMs = 0;          // 触发时刻时间戳
        static bool waitForClear = false;           // 等待检测清零，防止连续重复触发

        uint32_t now = millis();

        // 1. 已有舵机在工作中 → 检查是否到归位时间
        if (activeServo != 0)
        {
            if (now - triggerTimeMs >= SERVO_HOLD_MS)
            {
                setServoAngle(0);
                setServoAngle2(0);
                setServoAngle3(0);
                activeServo = 0;
                waitForClear = true;
            }
            return;
        }

        // 2. 刚归位，等待识别结果清零（防重复触发）
        if (waitForClear)
        {
            if (!g_lastAiDetected)
                waitForClear = false;
            return;
        }

        // 3. 空闲 → 检查是否有新目标
        if (!g_lastAiDetected)
            return;

        if (g_aiConfThreshold > 0.0f && g_lastAiConf < g_aiConfThreshold)
            return;

        const bool isBattery = (strcmp(g_lastAiLabel, "Battery") == 0);
        const bool isMobile = (strcmp(g_lastAiLabel, "MobilePhone") == 0);
        const bool isAccessory = (strcmp(g_lastAiLabel, "Charger") == 0 ||
                                  strcmp(g_lastAiLabel, "Earphone") == 0);

        if (isBattery)
        {
            setServoAngle(180); // servo1 → 电池仓
            activeServo = 1;
            triggerTimeMs = now;
        }
        else if (isMobile)
        {
            setServoAngle2(180); // servo2 → 手机仓
            activeServo = 2;
            triggerTimeMs = now;
        }
        else if (isAccessory)
        {
            setServoAngle3(180); // servo3 → 数码配件仓
            activeServo = 3;
            triggerTimeMs = now;
        }
        // 其他未知标签不触发任何舵机
    }

    void updateWeightsAndAlarm()
    {
        // 周期读取三路称重值，并同步到 OLED。
        // 三仓任一超过满载阈值即触发声光告警，全部低于回差后解除。
        g_currentWeight1 = static_cast<int32_t>(getWeight());
        g_currentWeight2 = static_cast<int32_t>(getWeight2());
        g_currentWeight3 = static_cast<int32_t>(getWeight3());

        setCurrentWeights(g_currentWeight1, g_currentWeight2, g_currentWeight3);

        // 满载告警：任一仓位超过阈值即触发，全部低于（阈值 - 回差）才解除。
        // 回差设计避免重量在阈值附近抖动导致告警反复开关。
        const bool anyFull = (g_currentWeight1 >= g_fullWeightG) || (g_currentWeight2 >= g_fullWeightG) || (g_currentWeight3 >= g_fullWeightG);
        const bool allBelow = (g_currentWeight1 < (g_fullWeightG - WARNING_RELEASE_HYSTERESIS_G)) && (g_currentWeight2 < (g_fullWeightG - WARNING_RELEASE_HYSTERESIS_G)) && (g_currentWeight3 < (g_fullWeightG - WARNING_RELEASE_HYSTERESIS_G));

        if (!g_warningActive && anyFull)
        {
            g_warningActive = true;
            digitalWrite(WARNING_LIGHT_PIN, HIGH); // 点亮告警灯
            buzzerOn();                            // 开启蜂鸣器
        }
        else if (g_warningActive && allBelow)
        {
            g_warningActive = false;
            digitalWrite(WARNING_LIGHT_PIN, LOW); // 关闭告警灯
            buzzerOff();                          // 关闭蜂鸣器
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
        // 第一个 false：不关闭底层 WiFi 驱动；第二个 false：不清除已保存的 SSID/密码
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

        // 构造三个仓位的上报数据包：{当前重量(g), 占满百分比(0~100%), 是否满载}
        const BoxBinData bin1 = {g_currentWeight1, clampPercent(g_currentWeight1), g_currentWeight1 >= g_fullWeightG};
        const BoxBinData bin2 = {g_currentWeight2, clampPercent(g_currentWeight2), g_currentWeight2 >= g_fullWeightG};
        const BoxBinData bin3 = {g_currentWeight3, clampPercent(g_currentWeight3), g_currentWeight3 >= g_fullWeightG};

        oneNetMqttUploadProperties(bin1, bin2, bin3, g_fullWeightG, g_aiConfThreshold);
    }
} // namespace

void setup()
{
    Serial.begin(115200);

    // 从 NVS 加载用户上次保存的满载阈值，需在 initOled() 之前完成，
    // 确保 OLED 启动时显示正确的满载基准。
    gPrefs.begin("sysconf", false);
    g_fullWeightG = gPrefs.getInt("full_w", 1000);
    g_aiConfThreshold = gPrefs.getFloat("ai_conf", 0.0f);
    Serial.printf("[CONFIG] Loaded overflow_threshold_g=%d g, ai_conf_threshold=%.4f from NVS\n",
                  g_fullWeightG, g_aiConfThreshold);

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
}

void loop()
{
    const uint32_t now = millis(); // 本次 loop 的时间戳，统一传给需要计时的函数，避免多次调用 millis() 产生不一致

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
