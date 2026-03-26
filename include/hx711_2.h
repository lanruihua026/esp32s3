#ifndef HX711_2_H
#define HX711_2_H

#include <Arduino.h>

/**
 * @brief 初始化第二个 HX711 称重模块（DT=GPIO11, SCK=GPIO12）
 *
 * 包含引脚初始化、预热、丢弃前几帧不稳定数据与去皮。
 * @return true 初始化成功；false 初始化失败（数据未就绪）
 */
bool setupHX711_2();

/**
 * @brief 获取第二个 HX711 当前重量（克）
 * @return 当前重量（g）
 */
float getWeight2();

/**
 * @brief 使用已知砝码对第二个 HX711 执行校准
 * @param knownWeight 已知重量（g）
 */
void calibrateScale2(float knownWeight);

/**
 * @brief 第二个 HX711 去皮（将当前读数设为零点）
 */
void tareScale2();

/**
 * @brief 手动设置第二个 HX711 的校准因子
 * @param factor 校准因子（raw / g）
 */
void setCalibrationFactor2(float factor);

void hx711GetProbeResult2(float *raw, int *validCount);

#endif // HX711_2_H
