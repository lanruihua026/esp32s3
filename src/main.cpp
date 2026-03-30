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
 * 初始化顺序：指示器 → 摄像头串口 → OLED → 尽早 WiFi.begin → HX711 → WiFi 等待 → 舵机 → 按键 → MQTT
 */

// ==================== OneNET 云平台配置 ====================
// 换设备或换账号时只改这里，然后重新烧录。
#define ONENET_PRODUCT_ID    "f45hkc7xC7"
#define ONENET_DEVICE_NAME   "Box1"
#define ONENET_BASE64_KEY    "T0R5ejYyM1JrT2VuczBkZllINmZuazRicEMxc29xcnk="
// Token 过期时间戳（Unix 秒），当前值对应 2030-01-01，到期前需更新并重新烧录
#define ONENET_TOKEN_EXPIRE_AT  1893456000UL

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "buttonControl.h"
#include "buzzer.h"
#include "connectToWiFi.h"
#include "hx711.h"
#include "hx711_2.h"
#include "hx711_3.h"
#include "hx711BootConfig.h"
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

    // HX711 出厂默认校准系数（raw ADC 差值 / 克），使用 216g 砝码标定。
    // 仅作启动回退值：若 NVS 中已保存过校准系数（hx1_scale/hx2_scale/hx3_scale），则优先使用 NVS 中的值。
    // 修改方法：new_factor = old_factor × (当前显示值 / 实际重量)
    // 校准修正：B2 实测 234g → 419.8×(234/216)≈454.5；B3 实测 197g → 455.5×(197/216)≈415.6
    constexpr float HX711_CAL_FACTOR_1 = 452.3f; // 1 号仓（GPIO1/2）  实测 215-216g，无需调整
    constexpr float HX711_CAL_FACTOR_2 = 454.5f; // 2 号仓（GPIO11/12）修正前 419.8，实测偏高 ~8.3%
    constexpr float HX711_CAL_FACTOR_3 = 415.6f; // 3 号仓（GPIO13/14）修正前 455.5，实测偏低 ~8.8%
    // 开机恢复零点后，若当前载荷低于该阈值，认为仓位接近空载，自动重新去皮修正漂移。
    constexpr float HX711_BOOT_EMPTY_THRESHOLD_G = 50.0f;

    // OneNET 配置宏定义在文件顶部（#define），此处直接使用。

    // NVS 持久化存储，用于保存满载阈值与 AI 置信度阈值，跨重启保持用户设置。
    Preferences gPrefs;
    bool g_prefsOk = false;

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
    // 串口标定流程状态：每路需要先 TARE，再允许 CAL，避免零点与跨度错配。
    bool g_calReadyCh1 = false;
    bool g_calReadyCh2 = false;
    bool g_calReadyCh3 = false;

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

    /** 在下发阈值变更后立即上报物模型，使平台「最新数据」与网页轮询尽快一致 */
    void reportPropertiesNow();

    /** 去掉标签尾部空白，避免 strcmp 与 YOLO 类名不一致 */
    void trimLabelInPlace(char *s)
    {
        if (s == nullptr || s[0] == '\0')
        {
            return;
        }
        size_t n = strlen(s);
        while (n > 0 && (s[n - 1] == ' ' || s[n - 1] == '\t' || s[n - 1] == '\r'))
        {
            s[--n] = '\0';
        }
    }

    /** 与训练侧类名比对时忽略大小写，避免 Battery / battery 导致不触发 */
    bool labelEqualsCi(const char *a, const char *b)
    {
        if (a == nullptr || b == nullptr)
        {
            return false;
        }
        while (*a != '\0' && *b != '\0')
        {
            const unsigned char ca = static_cast<unsigned char>(*a++);
            const unsigned char cb = static_cast<unsigned char>(*b++);
            if (std::tolower(ca) != std::tolower(cb))
            {
                return false;
            }
        }
        return *a == *b;
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
        setHx711ChannelReady(g_hx711_1InitOk, g_hx711_2InitOk, g_hx711_3InitOk);
        setAiConfThreshold(g_aiConfThreshold);
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
     * @brief 恢复或设置单路 HX711 的校准系数（calibration_factor）
     *
     * 策略：
     * - 若 NVS 中已保存过该路的系数（isKey 返回 true），优先使用 NVS 值，
     *   使运行时通过串口命令标定后写入的系数重启仍可生效。
     * - 若 NVS 中没有保存（首次上电），使用编译期默认值并写入 NVS，
     *   后续只需通过串口命令重新标定，无需再改常量重新烧录。
     *
     * @param nvsKey       NVS 中该路系数的键名（如 "hx1_scale"）
     * @param setScale     写入驱动校准系数的函数
     * @param defaultScale 编译期默认系数（HX711_CAL_FACTOR_*）
     * @param chLabel      日志用通道标识
     */
    static void restoreOrLoadScaleChannel(
        const char *nvsKey,
        void (*setScale)(float),
        float defaultScale,
        const char *chLabel)
    {
        if (!g_prefsOk)
        {
            setScale(defaultScale);
            Serial.printf("[HX711] %s: NVS unavailable, default calibration_factor=%.2f\n", chLabel, defaultScale);
            return;
        }
        if (gPrefs.isKey(nvsKey))
        {
            float saved = gPrefs.getFloat(nvsKey, defaultScale);
            setScale(saved);
            Serial.printf("[HX711] %s: loaded calibration_factor=%.2f from NVS\n", chLabel, saved);
        }
        else
        {
            setScale(defaultScale);
            gPrefs.putFloat(nvsKey, defaultScale);
            Serial.printf("[HX711] %s: using default calibration_factor=%.2f, saved to NVS\n", chLabel, defaultScale);
        }
    }

    /**
     * @brief 对单路 HX711 执行“恢复旧零点 + 条件自动去皮”并回写 NVS
     *
     * 策略：
     * - 若 NVS 中存在旧零点：先恢复，再测当前载荷；低于空载阈值则自动去皮并回写。
     * - 若 NVS 中不存在零点：视为首次启动，直接去皮并写入 NVS。
     * - 若采样无效或去皮失败：保留旧零点，不覆盖 NVS。
     */
    static void restoreOrTareChannel(
        const char *nvsKey,
        void (*setOffset)(float),
        bool (*doTare)(),
        float (*getOffset)(),
        float (*getLoadMag)(),
        const char *chLabel)
    {
        if (!g_prefsOk)
        {
            Serial.printf("[HX711] %s: NVS unavailable, tare in RAM only\n", chLabel);
            if (doTare())
            {
                Serial.printf("[HX711] %s: tare OK, offset not persisted\n", chLabel);
            }
            return;
        }
        if (gPrefs.isKey(nvsKey))
        {
            float savedOffset = gPrefs.getFloat(nvsKey, 0.0f);
            setOffset(savedOffset);
            float loadMagnitude = getLoadMag();
            Serial.printf("[HX711] %s: restored zero_offset=%.1f, load_abs=%.1fg\n",
                          chLabel, savedOffset, loadMagnitude);

            if (loadMagnitude >= 0.0f && loadMagnitude < HX711_BOOT_EMPTY_THRESHOLD_G)
            {
                if (doTare())
                {
                    gPrefs.putFloat(nvsKey, getOffset());
                    Serial.printf("[HX711] %s: near-empty, re-tared and saved zero_offset=%.1f\n",
                                  chLabel, getOffset());
                }
                else
                {
                    Serial.printf("[HX711] %s: near-empty but tare failed, keeping restored offset\n", chLabel);
                }
            }
            else if (loadMagnitude < 0.0f)
            {
                Serial.printf("[HX711] %s: load probe invalid, keeping restored offset\n", chLabel);
            }
            else
            {
                Serial.printf("[HX711] %s: bin has load (%.1fg >= %.1fg), keeping restored offset\n",
                              chLabel, loadMagnitude, HX711_BOOT_EMPTY_THRESHOLD_G);
            }
        }
        else
        {
            Serial.printf("[HX711] %s: no saved zero offset, first boot tare\n", chLabel);
            if (doTare())
            {
                gPrefs.putFloat(nvsKey, getOffset());
                Serial.printf("[HX711] %s: first-boot tare done, saved zero_offset=%.1f\n",
                              chLabel, getOffset());
            }
            else
            {
                Serial.printf("[HX711] %s: first-boot tare failed, zero offset unchanged\n", chLabel);
            }
        }
    }

    /**
     * @brief 初始化 HX711 重量传感器模块
     */
    void initHx711Modules()
    {
        // 依次初始化三路重量传感器：
        // 1) 恢复校准系数（优先 NVS）
        // 2) 恢复零点，必要时条件去皮
        // 三路全部可用才认为称重模块整体正常。
        setInitModuleStatus(INIT_MODULE_HX711, INIT_RUNNING, "Probe");

        if (HX711_BOOT_STRATEGY == 1)
        {
            showBootProgress(18, "HX711 warm");
            delay(HX711_GLOBAL_WARMUP_MS);
        }

        showBootProgress(20, "HX711_1");
        g_hx711_1InitOk = setupHX711();
        showBootProgress(28, "HX711_1 Cal");
        if (g_hx711_1InitOk)
        {
            restoreOrLoadScaleChannel("hx1_scale", setCalibrationFactor, HX711_CAL_FACTOR_1, "CH1");
            restoreOrTareChannel("hx1_zero", setZeroOffset, tareScale, getZeroOffset, getLoadMagnitude, "CH1");
        }

        showBootProgress(32, "HX711_2");
        g_hx711_2InitOk = setupHX711_2();
        showBootProgress(36, "HX711_2 Cal");
        if (g_hx711_2InitOk)
        {
            restoreOrLoadScaleChannel("hx2_scale", setCalibrationFactor2, HX711_CAL_FACTOR_2, "CH2");
            restoreOrTareChannel("hx2_zero", setZeroOffset2, tareScale2, getZeroOffset2, getLoadMagnitude2, "CH2");
        }

        showBootProgress(38, "HX711_3");
        g_hx711_3InitOk = setupHX711_3();
        showBootProgress(40, "HX711_3 Cal");
        if (g_hx711_3InitOk)
        {
            restoreOrLoadScaleChannel("hx3_scale", setCalibrationFactor3, HX711_CAL_FACTOR_3, "CH3");
            restoreOrTareChannel("hx3_zero", setZeroOffset3, tareScale3, getZeroOffset3, getLoadMagnitude3, "CH3");
        }

        setInitModuleStatus(INIT_MODULE_HX711, allHx711Ready() ? INIT_OK : INIT_ERROR, allHx711Ready() ? "Ready" : "Check");
    }

    /**
     * @brief 初始化 WiFi 模块，并在启动阶段等待连接（有超时机制）
     */
    void initWiFiWithTimeout()
    {
        // WiFi 在 initOled() 后已通过 startWiFiConnect() 发起；此处仅按总预算等待，
        // 预算从首次 WiFi.begin 起算，可与 HX711 初始化重叠。
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
    /**
     * @brief 初始化舵机模块，并执行自检
     */
    void initServoModule()
    {
        // 舵机属于执行器，放在网络之后初始化，
        // 这样供电更稳定，且自检动作更容易观察。
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
                             showBootProgress(72, line); });
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
     * 标准 OneJSON：params.xxx.value。
     * REST「设置设备属性」经平台转发后，常见为扁平：params.xxx 直接为数值（与 oneNet.js 请求体一致），
     * 仅解析 .value 会漏掉下发，导致云端与设备永不刷新。
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
            if (newThreshold < 100 || newThreshold > 5000)
            {
                Serial.printf("[CONFIG] overflow_threshold_g=%d out of range [100,5000], ignored\n", newThreshold);
            }
            else
            {
                g_fullWeightG = newThreshold;
                if (g_prefsOk)
                {
                    gPrefs.putInt("full_w", newThreshold);
                }
                setFullWeight(g_fullWeightG);
                Serial.printf("[CONFIG] overflow_threshold_g updated to %d g\n", newThreshold);
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
            if (newAi < 0.0f || newAi > 1.0f)
            {
                Serial.printf("[CONFIG] ai_conf_threshold=%.4f out of range [0,1], ignored\n", newAi);
            }
            else
            {
                g_aiConfThreshold = newAi;
                if (g_prefsOk)
                {
                    gPrefs.putFloat("ai_conf", newAi);
                }
                Serial.printf("[CONFIG] ai_conf_threshold updated to %.4f\n", newAi);
                changed = true;
            }
        }

        if (changed)
        {
            reportPropertiesNow();
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
            ONENET_TOKEN_EXPIRE_AT,    // Token 过期时间戳（统一在上方常量区维护）
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
            label[sizeof(label) - 1] = '\0';
            trimLabelInPlace(label);

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
        // 同一时刻仅一路为 180°，其余强制 0°，避免多路同时受力或误动。
        // 再触发条件：收到 NONE、识别类别变化、或超时（CAM 持续 DET 时不会发 NONE，仅靠 NONE 会永久锁死）。
        // 连续帧确认：需连续收到 SERVO_CONFIRM_FRAMES 帧相同标签才真正触发舵机，防止单帧误判。
        // 仓位分配：
        //   Battery（电池）          → servo1 (GPIO7)  电池仓
        //   MobilePhone（手机）      → servo2 (GPIO8)  手机仓
        //   Charger / Earphone       → servo3 (GPIO16) 数码配件仓
        static const uint32_t SERVO_HOLD_MS = 2000;      // 舵机保持 180° 的时长（毫秒）
        static const uint32_t REARM_SAME_VIEW_MS = 3500; // 同类持续在画面中时的再允许分拣间隔（略大于 CAM 推理周期）
        static const int SERVO_CONFIRM_FRAMES = 2;       // 触发所需连续正检帧数（≥2 可抑制单帧误检）
        static int activeServo = 0;                      // 当前激活的舵机（0=无，1/2/3）
        static uint32_t triggerTimeMs = 0;               // 触发时刻时间戳
        static bool waitForRearm = false;                // 归位后等待允许再次分拣
        static char labelAtLastSort[sizeof(g_lastAiLabel)] = "";
        static uint32_t sortCompletedMs = 0; // 上次归位完成时刻
        static int confirmCount = 0;                              // 当前连续帧计数
        static char confirmLabel[sizeof(g_lastAiLabel)] = "";    // 待确认的标签

        uint32_t now = millis();

        // 1. 已有舵机在工作中 → 检查是否到归位时间
        if (activeServo != 0)
        {
            if (now - triggerTimeMs >= SERVO_HOLD_MS)
            {
                if (activeServo == 1)
                {
                    setServoAngle(0);
                    Serial.printf("[SERVO] HOME ch=1 GPIO%d angle=0 deg (hold %lu ms done)\n", SERVO_PIN,
                                  static_cast<unsigned long>(SERVO_HOLD_MS));
                }
                else if (activeServo == 2)
                {
                    setServoAngle2(0);
                    Serial.printf("[SERVO] HOME ch=2 GPIO%d angle=0 deg (hold %lu ms done)\n", SERVO_PIN_2,
                                  static_cast<unsigned long>(SERVO_HOLD_MS));
                }
                else if (activeServo == 3)
                {
                    setServoAngle3(0);
                    Serial.printf("[SERVO] HOME ch=3 GPIO%d angle=0 deg (hold %lu ms done)\n", SERVO_PIN_3,
                                  static_cast<unsigned long>(SERVO_HOLD_MS));
                }
                activeServo = 0;
                waitForRearm = true;
                strncpy(labelAtLastSort, g_lastAiLabel, sizeof(labelAtLastSort) - 1);
                labelAtLastSort[sizeof(labelAtLastSort) - 1] = '\0';
                sortCompletedMs = now;
            }
            return;
        }

        // 2. 刚归位：允许再次分拣当且仅当（自动分拣常见三种情况）
        if (waitForRearm)
        {
            if (!g_lastAiDetected)
            {
                waitForRearm = false;
            }
            else if (!labelEqualsCi(g_lastAiLabel, labelAtLastSort))
            {
                // 模型在下一帧判成另一类：视为新一次分拣，不必等 NONE
                waitForRearm = false;
            }
            else if ((now - sortCompletedMs) >= REARM_SAME_VIEW_MS)
            {
                // 画面里仍是同类且一直无 NONE：超时解锁，避免永远不触发
                waitForRearm = false;
            }
            else
            {
                return;
            }
        }

        // 3. 空闲 → 连续帧确认后才触发舵机，防止单帧误判
        if (!g_lastAiDetected || (g_aiConfThreshold > 0.0f && g_lastAiConf < g_aiConfThreshold))
        {
            // 未检测到或置信度不足：清零确认计数
            confirmCount = 0;
            confirmLabel[0] = '\0';
            return;
        }

        // 与上一帧标签相同才累计计数，标签切换则重置
        if (!labelEqualsCi(g_lastAiLabel, confirmLabel))
        {
            confirmCount = 1;
            strncpy(confirmLabel, g_lastAiLabel, sizeof(confirmLabel) - 1);
            confirmLabel[sizeof(confirmLabel) - 1] = '\0';
            Serial.printf("[SERVO] CONFIRM reset label=%s (need %d frames)\n", confirmLabel, SERVO_CONFIRM_FRAMES);
            return;
        }
        confirmCount++;
        if (confirmCount < SERVO_CONFIRM_FRAMES)
        {
            Serial.printf("[SERVO] CONFIRM accumulate label=%s count=%d/%d\n",
                          confirmLabel, confirmCount, SERVO_CONFIRM_FRAMES);
            return;
        }
        // 达到确认帧数 → 触发分拣，重置计数器
        confirmCount = 0;
        confirmLabel[0] = '\0';

        const bool isBattery = labelEqualsCi(g_lastAiLabel, "Battery");
        const bool isMobile = labelEqualsCi(g_lastAiLabel, "MobilePhone");
        const bool isAccessory = labelEqualsCi(g_lastAiLabel, "Charger") ||
                                 labelEqualsCi(g_lastAiLabel, "Earphone");

        if (isBattery)
        {
            setServoAngle(180);
            setServoAngle2(0);
            setServoAngle3(0);
            activeServo = 1;
            triggerTimeMs = now;
            Serial.printf("[SERVO] TRIGGER ch=1 GPIO%d angle=180 deg label=%s conf=%.1f%% (hold %lu ms)\n",
                          SERVO_PIN, g_lastAiLabel, g_lastAiConf * 100.0f,
                          static_cast<unsigned long>(SERVO_HOLD_MS));
        }
        else if (isMobile)
        {
            setServoAngle(0);
            setServoAngle2(180);
            setServoAngle3(0);
            activeServo = 2;
            triggerTimeMs = now;
            Serial.printf("[SERVO] TRIGGER ch=2 GPIO%d angle=180 deg label=%s conf=%.1f%% (hold %lu ms)\n",
                          SERVO_PIN_2, g_lastAiLabel, g_lastAiConf * 100.0f,
                          static_cast<unsigned long>(SERVO_HOLD_MS));
        }
        else if (isAccessory)
        {
            setServoAngle(0);
            setServoAngle2(0);
            setServoAngle3(180);
            activeServo = 3;
            triggerTimeMs = now;
            Serial.printf("[SERVO] TRIGGER ch=3 GPIO%d angle=180 deg label=%s conf=%.1f%% (hold %lu ms)\n",
                          SERVO_PIN_3, g_lastAiLabel, g_lastAiConf * 100.0f,
                          static_cast<unsigned long>(SERVO_HOLD_MS));
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

    void reportPropertiesNow()
    {
        if (!oneNetMqttConnected())
        {
            return;
        }
        const BoxBinData bin1 = {g_currentWeight1, clampPercent(g_currentWeight1), g_currentWeight1 >= g_fullWeightG};
        const BoxBinData bin2 = {g_currentWeight2, clampPercent(g_currentWeight2), g_currentWeight2 >= g_fullWeightG};
        const BoxBinData bin3 = {g_currentWeight3, clampPercent(g_currentWeight3), g_currentWeight3 >= g_fullWeightG};
        oneNetMqttUploadProperties(bin1, bin2, bin3, g_fullWeightG, g_aiConfThreshold);
    }

    // ===== 串口标定命令处理 =====
    // 通过 USB 串口（115200 波特率）对三路 HX711 执行去皮和校准，
    // 标定结果立即持久化到 NVS，重启后无需重新烧录仍可保留。
    //
    // 支持的命令（发送后按回车，大小写均可）：
    //   TARE1 / TARE2 / TARE3              空仓去皮，请先确认该仓为空载稳定后发送
    //   CAL1:<重量g> / CAL2:<重量g> / CAL3:<重量g>
    //                                       放已知砝码后校准，例如 "CAL2:216"
    //   STATUS                              打印三路当前系数、零点和实时重量
    //
    // 标准校准流程：
    //   1. 空仓 → 发送 TARE1（去皮并保存零点到 NVS）
    //   2. 放已知重量砝码（如 216g）→ 发送 CAL1:216（保存新系数到 NVS）
    //   3. 重启设备 → 发送 STATUS 确认系数已恢复

    char g_serialCmdBuf[64] = {0};
    uint8_t g_serialCmdPos = 0;

    void handleSerialCommand(const char *cmd)
    {
        if (cmd == nullptr || cmd[0] == '\0')
            return;

        // TARE1 / TARE2 / TARE3
        if (strcmp(cmd, "TARE1") == 0)
        {
            Serial.println("[CAL] TARE1: taring CH1...");
            g_calReadyCh1 = false;
            if (tareScale())
            {
                if (g_prefsOk)
                {
                    gPrefs.putFloat("hx1_zero", getZeroOffset());
                }
                g_calReadyCh1 = true;
                Serial.printf(g_prefsOk ? "[CAL] TARE1 OK: zero_offset=%.1f saved\n"
                                        : "[CAL] TARE1 OK: zero_offset=%.1f (NVS unavailable, not saved)\n",
                              getZeroOffset());
            }
            else
            {
                Serial.println("[CAL] TARE1 FAIL: insufficient valid samples");
            }
            return;
        }
        if (strcmp(cmd, "TARE2") == 0)
        {
            Serial.println("[CAL] TARE2: taring CH2...");
            g_calReadyCh2 = false;
            if (tareScale2())
            {
                if (g_prefsOk)
                {
                    gPrefs.putFloat("hx2_zero", getZeroOffset2());
                }
                g_calReadyCh2 = true;
                Serial.printf(g_prefsOk ? "[CAL] TARE2 OK: zero_offset=%.1f saved\n"
                                        : "[CAL] TARE2 OK: zero_offset=%.1f (NVS unavailable, not saved)\n",
                              getZeroOffset2());
            }
            else
            {
                Serial.println("[CAL] TARE2 FAIL: insufficient valid samples");
            }
            return;
        }
        if (strcmp(cmd, "TARE3") == 0)
        {
            Serial.println("[CAL] TARE3: taring CH3...");
            g_calReadyCh3 = false;
            if (tareScale3())
            {
                if (g_prefsOk)
                {
                    gPrefs.putFloat("hx3_zero", getZeroOffset3());
                }
                g_calReadyCh3 = true;
                Serial.printf(g_prefsOk ? "[CAL] TARE3 OK: zero_offset=%.1f saved\n"
                                        : "[CAL] TARE3 OK: zero_offset=%.1f (NVS unavailable, not saved)\n",
                              getZeroOffset3());
            }
            else
            {
                Serial.println("[CAL] TARE3 FAIL: insufficient valid samples");
            }
            return;
        }

        // CAL1:<g> / CAL2:<g> / CAL3:<g>
        if (strncmp(cmd, "CAL1:", 5) == 0)
        {
            float w = atof(cmd + 5);
            if (w <= 0.0f)
            {
                Serial.println("[CAL] CAL1 FAIL: invalid weight");
                return;
            }
            if (!g_calReadyCh1)
            {
                Serial.println("[CAL] CAL1 FAIL: run TARE1 first in current session");
                return;
            }
            Serial.printf("[CAL] CAL1: calibrating CH1 with %.1fg...\n", w);
            if (calibrateScale(w))
            {
                if (g_prefsOk)
                {
                    gPrefs.putFloat("hx1_scale", getCalibrationFactor());
                }
                g_calReadyCh1 = false;
                Serial.printf(g_prefsOk ? "[CAL] CAL1 OK: factor=%.2f saved to NVS\n"
                                        : "[CAL] CAL1 OK: factor=%.2f (NVS unavailable, not saved)\n",
                              getCalibrationFactor());
            }
            else
            {
                Serial.println("[CAL] CAL1 FAIL: calibration rejected (sensor not ready or samples invalid)");
            }
            return;
        }
        if (strncmp(cmd, "CAL2:", 5) == 0)
        {
            float w = atof(cmd + 5);
            if (w <= 0.0f)
            {
                Serial.println("[CAL] CAL2 FAIL: invalid weight");
                return;
            }
            if (!g_calReadyCh2)
            {
                Serial.println("[CAL] CAL2 FAIL: run TARE2 first in current session");
                return;
            }
            Serial.printf("[CAL] CAL2: calibrating CH2 with %.1fg...\n", w);
            if (calibrateScale2(w))
            {
                if (g_prefsOk)
                {
                    gPrefs.putFloat("hx2_scale", getCalibrationFactor2());
                }
                g_calReadyCh2 = false;
                Serial.printf(g_prefsOk ? "[CAL] CAL2 OK: factor=%.2f saved to NVS\n"
                                        : "[CAL] CAL2 OK: factor=%.2f (NVS unavailable, not saved)\n",
                              getCalibrationFactor2());
            }
            else
            {
                Serial.println("[CAL] CAL2 FAIL: calibration rejected (sensor not ready or samples invalid)");
            }
            return;
        }
        if (strncmp(cmd, "CAL3:", 5) == 0)
        {
            float w = atof(cmd + 5);
            if (w <= 0.0f)
            {
                Serial.println("[CAL] CAL3 FAIL: invalid weight");
                return;
            }
            if (!g_calReadyCh3)
            {
                Serial.println("[CAL] CAL3 FAIL: run TARE3 first in current session");
                return;
            }
            Serial.printf("[CAL] CAL3: calibrating CH3 with %.1fg...\n", w);
            if (calibrateScale3(w))
            {
                if (g_prefsOk)
                {
                    gPrefs.putFloat("hx3_scale", getCalibrationFactor3());
                }
                g_calReadyCh3 = false;
                Serial.printf(g_prefsOk ? "[CAL] CAL3 OK: factor=%.2f saved to NVS\n"
                                        : "[CAL] CAL3 OK: factor=%.2f (NVS unavailable, not saved)\n",
                              getCalibrationFactor3());
            }
            else
            {
                Serial.println("[CAL] CAL3 FAIL: calibration rejected (sensor not ready or samples invalid)");
            }
            return;
        }

        // STATUS
        if (strcmp(cmd, "STATUS") == 0)
        {
            Serial.println("[CAL] === HX711 Status ===");
            Serial.printf("[CAL] CH1: factor=%.2f  zero=%.1f  weight=%dg\n",
                          getCalibrationFactor(), getZeroOffset(), g_currentWeight1);
            Serial.printf("[CAL] CH2: factor=%.2f  zero=%.1f  weight=%dg\n",
                          getCalibrationFactor2(), getZeroOffset2(), g_currentWeight2);
            Serial.printf("[CAL] CH3: factor=%.2f  zero=%.1f  weight=%dg\n",
                          getCalibrationFactor3(), getZeroOffset3(), g_currentWeight3);
            return;
        }

        Serial.printf("[CAL] Unknown command: %s\n", cmd);
        Serial.println("[CAL] Commands: TARE1/2/3  CAL1:<g>/CAL2:<g>/CAL3:<g>  STATUS");
    }

    void pollSerialCommands()
    {
        while (Serial.available())
        {
            char c = static_cast<char>(Serial.read());
            if (c == '\r')
                continue;
            if (c == '\n')
            {
                g_serialCmdBuf[g_serialCmdPos] = '\0';
                for (uint8_t i = 0; i < g_serialCmdPos; i++)
                {
                    if (g_serialCmdBuf[i] >= 'a' && g_serialCmdBuf[i] <= 'z')
                        g_serialCmdBuf[i] -= 32; // 转大写，命令大小写不敏感
                }
                if (g_serialCmdPos > 0)
                    handleSerialCommand(g_serialCmdBuf);
                g_serialCmdPos = 0;
                continue;
            }
            if (g_serialCmdPos < sizeof(g_serialCmdBuf) - 1)
                g_serialCmdBuf[g_serialCmdPos++] = c;
            else
                g_serialCmdPos = 0; // 行过长，丢弃重新接收
        }
    }
} // namespace

void setup()
{
    Serial.begin(115200);

    // 从 NVS 加载用户上次保存的满载阈值，需在 initOled() 之前完成，
    // 确保 OLED 启动时显示正确的满载基准。
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
    Serial.printf("[CONFIG] overflow_threshold_g=%d g, ai_conf_threshold=%.4f (%s)\n",
                  g_fullWeightG, g_aiConfThreshold, g_prefsOk ? "from NVS" : "defaults");

    // 初始化顺序遵循嵌入式常见思路：
    // 先基础外设，再显示，再传感器，再网络，再执行器，最后云端和交互。
    initBoardIndicators();
    initCameraUart();
    initOled();
    startWiFiConnect();
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
    pollSerialCommands(); // 处理串口标定命令（TARE/CAL/STATUS），不影响正常运行
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
