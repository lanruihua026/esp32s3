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

// 全局 OLED 对象（在 oledInit.cpp 中定义）
extern Adafruit_SSD1306 oledDisplay;

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

/**
 * @brief 显示开机进度条页面
 * @param progress 进度百分比（0~100）
 * @param statusText 当前阶段文本
 */
void showBootProgress(uint8_t progress, const char *statusText);

#endif
