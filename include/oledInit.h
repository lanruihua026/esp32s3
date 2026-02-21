#ifndef _OLED_INIT_H_
#define _OLED_INIT_H_

#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Wire.h>
#include <Arduino.h>
// OLED 屏幕与 I2C 引脚配置
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
#define SDA_PIN 4
#define SCL_PIN 5
extern Adafruit_SSD1306 oledDisplay; // 声明 OLED 显示对象

void setupOLED(); // 声明 OLED 初始化函数

#endif