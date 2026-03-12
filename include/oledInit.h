#ifndef _OLED_INIT_H_
#define _OLED_INIT_H_

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Wire.h>

// ===== OLED 硬件参数 =====
// 128x64 I2C OLED（SSD1306）
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
extern Adafruit_SSD1306 oledDisplay;

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
 * @brief 刷新运行态 OLED 页面
 *
 * 该函数应在 loop() 中持续调用，用于更新动画与状态页。
 */
void updateOLEDDisplay();

/**
 * @brief 设置“正在上传”状态（影响 OLED 状态行显示）
 */
void setUploadingStatus(bool isUploading);

/**
 * @brief 更新当前重量（影响 OLED 重量显示）
 */
void setCurrentWeight(int32_t weight);

/** * @brief 更新 ESP32-CAM 最新 AI 识别结果（供 AI 结果页显示）
 * @param detected  是否识别到目标
 * @param label     识别标签（字符串，最多 31 字符）
 * @param conf      置信度（0.0~1.0）
 * @param updateMs  本次识别时的 millis() 时间戳
 */
void setAiResult(bool detected, const char *label, float conf, uint32_t updateMs);

/**
 * @brief 切换 OLED 当前显示页面
 * @param page  0 = 综合信息页，1 = AI 识别结果页
 */
void setOledPage(uint8_t page);

/** * @brief 显示开机进度条页面
 * @param progress 进度百分比（0~100）
 * @param statusText 当前阶段文本
 */
void showBootProgress(uint8_t progress, const char *statusText);

#endif
