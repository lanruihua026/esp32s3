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

/**
 * @brief 读取 probe 阶段的原始值与有效采样数（用于 OLED 调试）
 * @param raw        输出：probe 均值（全部超时时为 0）
 * @param validCount 输出：5 次采样中成功读到数据的次数（0 = 全部超时）
 */
void hx711GetProbeResult(float *raw, int *validCount);

#endif // HX711_H
