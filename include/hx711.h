#ifndef HX711_H
#define HX711_H

#include <Arduino.h>

/**
 * @brief 初始化 HX711 称重模块
 *
 * 包含引脚初始化、预热、丢弃前几帧不稳定数据与去皮。
 * @return true 初始化成功；false 初始化失败（数据未就绪）
 */
bool setupHX711();

/**
 * @brief 获取当前重量（克）
 * @return 当前重量（g）
 */
float getWeight();

/**
 * @brief 使用已知砝码执行校准
 * @param knownWeight 已知重量（g）
 */
void calibrateScale(float knownWeight);

/**
 * @brief 去皮（将当前读数设为零点）
 */
void tareScale();

/**
 * @brief 手动设置校准因子
 * @param factor 校准因子（raw / g）
 */
void setCalibrationFactor(float factor);


#endif // HX711_H
