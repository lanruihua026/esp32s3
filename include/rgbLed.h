#ifndef _RGB_LED_H_
#define _RGB_LED_H_

#include <Arduino.h>

// ===== 板载 RGB LED 硬件参数 =====
// ESP32-S3-DevKitM-1 板载 WS2812B 接在 GPIO 48
#define RGB_LED_PIN 48
#define RGB_LED_COUNT 1
#define RGB_BRIGHTNESS 50 // 亮度（0~255），默认 50/255 避免过亮

/**
 * @brief 初始化板载 RGB LED（WS2812B）
 *
 * 执行内容：
 * 1. 初始化 NeoPixel 驱动。
 * 2. 默认关闭 LED。
 * 3. 设置初始亮度。
 */
void setupRgbLed();

/**
 * @brief 点亮 RGB LED（白色）
 */
void rgbLedOn();

/**
 * @brief 关闭 RGB LED
 */
void rgbLedOff();

/**
 * @brief 设置 RGB LED 自定义颜色
 * @param r 红色分量（0~255）
 * @param g 绿色分量（0~255）
 * @param b 蓝色分量（0~255）
 */
void setRgbLed(uint8_t r, uint8_t g, uint8_t b);

#endif // _RGB_LED_H_
