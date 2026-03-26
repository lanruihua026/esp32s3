#ifndef HX711_3_H
#define HX711_3_H

#include <Arduino.h>

/**
 * @brief 初始化第三个 HX711 称重模块（DT=GPIO13, SCK=GPIO14）
 *
 * 包含引脚初始化、预热、丢弃前几帧不稳定数据与去皮。
 * @return true 初始化成功；false 初始化失败（数据未就绪）
 */
bool setupHX711_3();

/**
 * @brief 获取第三个 HX711 当前重量（克）
 * @return 当前重量（g）
 */
float getWeight3();

/**
 * @brief 使用已知砝码对第三个 HX711 执行校准
 * @param knownWeight 已知重量（g）
 */
void calibrateScale3(float knownWeight);

/**
 * @brief 第三个 HX711 去皮（将当前读数设为零点）
 */
void tareScale3();

/**
 * @brief 手动设置第三个 HX711 的校准因子
 * @param factor 校准因子（raw / g）
 */
void setCalibrationFactor3(float factor);


#endif // HX711_3_H
