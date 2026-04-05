#ifndef _OLED_INIT_H_
#define _OLED_INIT_H_

#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Arduino.h>
#include <Wire.h>

// ===== OLED 硬件参数 =====
// 128x64 I2C OLED（SH1106）
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

// ESP32-S3 I2C 引脚
#define SDA_PIN 4
#define SCL_PIN 5

// ===== 业务阈值 =====
// HX711 传感器量程 5000g，实际最大载重量设定为 1000g
#define HX711_SENSOR_MAX 5000 // 传感器量程上限（g）

// 全局 OLED 对象（在 oledInit.cpp 中定义）
extern Adafruit_SH1106G oledDisplay;

// ===== 启动阶段状态 =====
enum InitModuleId : uint8_t
{
    INIT_MODULE_OLED = 0,
    INIT_MODULE_HX711,
    INIT_MODULE_WIFI,
    INIT_MODULE_SERVO,
    INIT_MODULE_BUTTON,
    INIT_MODULE_MQTT,
    INIT_MODULE_COUNT
};

enum InitModuleStatus : uint8_t
{
    INIT_PENDING = 0,
    INIT_RUNNING,
    INIT_OK,
    INIT_ERROR,
    INIT_TIMEOUT,
    INIT_SKIPPED
};

/**
 * @brief 设置满载阈值（影响 OLED 百分比计算与满载状态显示）
 * @param fw 满载重量（g），由 main.cpp 在 setup() 中传入
 */
void setFullWeight(int32_t fw);

/**
 * @brief 初始化 OLED 与 I2C
 *
 * 执行内容：
 * 1. 初始化 I2C 总线与速率。
 * 2. 初始化 SSD1306 驱动。
 * 3. 显示开机进度页初始状态。
 */
void setupOLED();

/**
 * @brief 查询 OLED 是否可用
 * @return true OLED 初始化成功；false OLED 初始化失败（已降级）
 */
bool isOLEDReady();

/**
 * @brief 刷新运行态 OLED 页面
 *
 * 该函数应在 loop() 中持续调用，用于更新动画与状态页。
 */
void updateOLEDDisplay();

/**
 * @brief 更新当前重量（兼容旧接口，仅更新1号仓）
 */
void setCurrentWeight(int32_t weight);

/**
 * @brief 更新三个仓格的当前重量（影响三仓重量页显示）
 */
void setCurrentWeights(int32_t weight1, int32_t weight2, int32_t weight3);

/**
 * @brief 更新 ESP32-CAM 最新 AI 识别结果
 * @param detected  是否识别到目标
 * @param label     识别标签（字符串，最多 31 字符）
 * @param conf      置信度（0.0~1.0）
 * @param updateMs  本次识别时的 millis() 时间戳
 * 业务含义：把摄像头最新识别结果同步到 OLED 的“识别结果页”。
 */
void setAiResult(bool detected, const char *label, float conf, uint32_t updateMs);

/**
 * AI 错误分类：识别页只保留两种用户可感知错误。
 * - AI_ERR_CAM_OFFLINE: 摄像头板离线，或摄像头侧 WiFi 未连接
 * - AI_ERR_SERVICE_OFFLINE: 摄像头在线，但推理后端不可用
 */
enum AiErrorKind : uint8_t {
    AI_ERR_NONE = 0,
    AI_ERR_CAM_OFFLINE,
    AI_ERR_SERVICE_OFFLINE
};

/**
 * @brief 更新 AI 错误态（例如推理服务未开启、网络异常等）
 * @param hasError 是否处于错误态
 * @param updateMs 本次错误更新的 millis() 时间戳
 * @param kind     错误分类（默认 AI_ERR_CAM_OFFLINE）
 * 业务含义：识别结果页在错误态时 Status 行显示 ERR，Label 行显示具体错误类型。
 */
void setAiError(bool hasError, uint32_t updateMs, AiErrorKind kind = AI_ERR_CAM_OFFLINE);

/**
 * @brief 切换 OLED 当前显示页面
 * @param page  0 = 系统状态页，1 = 三仓重量页，2 = 识别结果页
 */
void setOledPage(uint8_t page);

/**
 * @brief 向右循环切换 OLED 页面（0→1→2→0）
 */
void toggleOledPage();

/**
 * @brief 向左循环切换 OLED 页面（0←1←2←0）
 */
void prevOledPage();

/**
 * @brief 重置启动模块状态
 */
void resetInitModuleStatus();

/**
 * @brief 设置某个启动模块的状态与细节文本
 */
void setInitModuleStatus(InitModuleId module, InitModuleStatus status, const char *detail);

/**
 * @brief 设置运行态模块健康状态（用于主页面 ERROR 摘要）
 */
void setRuntimeHealth(bool wifiOk, bool wifiInitTimeout, bool hx711Ok, bool mqttOk);

/**
 * @brief 同步三路 HX711 初始化结果（重量页分路 ERR、状态页摘要）
 */
void setHx711ChannelReady(bool ch1, bool ch2, bool ch3);

/**
 * @brief 同步三路称重超量程状态（重量页超量程时显示 OVER 而非克数）
 * @param ch1/ch2/ch3 各路是否超过传感器量程上限（HX711_MAX_RANGE_G）
 */
void setWeightOverRange(bool ch1, bool ch2, bool ch3);

/**
 * @brief 同步 AI 置信度阈值（识别页 Thr，与 g_aiConfThreshold 一致）
 */
void setAiConfThreshold(float threshold);

/**
 * @brief 显示开机进度条页面
 * @param progress 进度百分比（0~100）
 * @param statusText 当前阶段文本
 * 业务含义：让用户看到系统当前初始化到了哪一步。
 */
void showBootProgress(uint8_t progress, const char *statusText);

#endif
