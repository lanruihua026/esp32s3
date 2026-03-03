#ifndef HX711_H
#define HX711_H

#include <Arduino.h>

/**
 * @brief 初始化 HX711 称重传感器
 */
void setupHX711();

/**
 * @brief 获取当前重量（克）
 * @return float 重量值
 */
float getWeight();

/**
 * @brief 校准传感器
 * @param knownWeight 已知的校准重量（克）
 */
void calibrateScale(float knownWeight);

/**
 * @brief 重新去皮（归零）
 */
void tareScale();

/**
 * @brief 设置校准因子（用于已有校准值时）
 * @param factor 校准因子
 */
void setCalibrationFactor(float factor);

#endif // HX711_H