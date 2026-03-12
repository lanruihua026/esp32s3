#include "rgbLed.h"
#include <Adafruit_NeoPixel.h>

// NeoPixel 驱动对象（单颗 WS2812B，GPIO 48）
static Adafruit_NeoPixel strip(RGB_LED_COUNT, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

void setupRgbLed()
{
    strip.begin();
    strip.setBrightness(RGB_BRIGHTNESS);
    strip.show(); // 上电默认关闭
    Serial.println("[RGB] LED initialized, off by default.");
}

void rgbLedOn()
{
    strip.setPixelColor(0, strip.Color(255, 255, 255)); // 白色
    strip.show();
    Serial.println("[RGB] LED ON");
}

void rgbLedOff()
{
    strip.setPixelColor(0, strip.Color(0, 0, 0)); // 熄灭
    strip.show();
    Serial.println("[RGB] LED OFF");
}

void setRgbLed(uint8_t r, uint8_t g, uint8_t b)
{
    strip.setPixelColor(0, strip.Color(r, g, b));
    strip.show();
    Serial.printf("[RGB] LED color R=%d G=%d B=%d\n", r, g, b);
}
